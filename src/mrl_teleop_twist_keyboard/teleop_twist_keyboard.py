#!/usr/bin/env python

from __future__ import print_function,unicode_literals, absolute_import

import roslib; roslib.load_manifest('mrl_teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

import xlib
import keysyms

if sys.version_info.major < 3:
    import glib
else:
    from gi.repository import GLib as glib

import threading
import warnings



# convenience wrappers
def coalesce_ranges(ranges):
    ranges = sorted(ranges, key=lambda x: x[0])
    ret = ranges[:1]
    for r in ranges[1:]:
        if ret[-1][1] < r[0] - 1:
            ret.append(r)
        else:
            ret[-1][1] = max(ret[-1][1], r[1])
    return ret


def record_context(dpy, ev_ranges, dev_ranges):
    ev_ranges = coalesce_ranges(ev_ranges)
    dev_ranges = coalesce_ranges(dev_ranges)

    specs = max(len(ev_ranges), len(dev_ranges))
    range_specs = (xlib.POINTER(xlib.XRecordRange) * specs)()

    for i in range(specs):
        range_specs[i] = xlib.XRecordAllocRange()
        if len(ev_ranges) > i:
            range_specs[i].contents.delivered_events.first = ev_ranges[i][0]
            range_specs[i].contents.delivered_events.last = ev_ranges[i][1]
        if len(dev_ranges) > i:
            range_specs[i].contents.device_events.first = dev_ranges[i][0]
            range_specs[i].contents.device_events.last = dev_ranges[i][1]

    rec_ctx = xlib.XRecordCreateContext(
        dpy, 0,
        xlib.byref(xlib.c_ulong(xlib.XRecordAllClients)), 1,
        range_specs, specs)

    for i in range(specs):
        xlib.XFree(range_specs[i])

    return rec_ctx


def record_enable(dpy, rec_ctx, callback):
    def intercept(data):
        if data.category != xlib.XRecordFromServer:
            return
        if data.client_swapped:
            warnings.warn("cannot handle swapped protocol data")
            return
        ev = xlib.XWireToEvent(dpy, data.data)
        callback(ev)

    def intercept_(_, data):
        intercept(data.contents)
        xlib.XRecordFreeData(data)

    proc = xlib.XRecordInterceptProc(intercept_)
    xlib.XRecordEnableContextAsync(dpy, rec_ctx, proc, None)
    return proc


def create_replay_window(dpy):
    win_attr = xlib.XSetWindowAttributes()
    win_attr.override_redirect = True
    win = xlib.XCreateWindow(dpy, xlib.XDefaultRootWindow(dpy),
                             0, 0, 1, 1, 0,
                             xlib.CopyFromParent, xlib.InputOnly, None,
                             xlib.CWOverrideRedirect,
                             xlib.byref(win_attr))
    return win


def phantom_release(dpy, kev):
    if not xlib.XPending(dpy):
        return False
    ev = xlib.XEvent()
    xlib.XPeekEvent(dpy, xlib.byref(ev))
    return (ev.type == xlib.KeyPress and \
            ev.xkey.state == kev.state and \
            ev.xkey.keycode == kev.keycode and \
            ev.xkey.time == kev.time)


def keysym_to_unicode(keysym):
    if 0x01000000 <= keysym <= 0x0110FFFF:
        return unichr(keysym - 0x01000000)
    keydata = keysyms.KEYSYMS.get(keysym)
    if keydata is not None:
        return keydata[0]
    return None



class KeyData():
    def __init__(self, pressed=None, filtered=None, repeated=None,
                 string=None, keysym=None, status=None, symbol=None,
                 mods_mask=None, modifiers=None):
        self.pressed = pressed
        self.filtered = filtered
        self.repeated = repeated
        self.string = string
        self.keysym = keysym
        self.status = status
        self.symbol = symbol
        self.mods_mask = mods_mask
        self.modifiers = modifiers


class InputType:
    keyboard = 0b001
    button   = 0b010
    movement = 0b100
    all      = 0b111



class InputListener(threading.Thread):
    def __init__(self, callback, input_types=InputType.all, kbd_compose=True, kbd_translate=True):
        super(InputListener, self).__init__()
        self.callback = callback
        self.input_types = input_types
        self.kbd_compose = kbd_compose
        self.kbd_translate = kbd_translate
        self.lock = threading.Lock()
        self._stop = True
        self.error = None


    def _event_received(self, ev):
        if xlib.KeyPress <= ev.type <= xlib.MotionNotify:
            xlib.XSendEvent(self.replay_dpy, self.replay_win, False, 0, ev)
        elif ev.type in [xlib.FocusIn, xlib.FocusOut]:
            # Forward the event as a custom message in the same queue instead
            # of resetting the XIC directly, in order to preserve queued events
            fwd_ev = xlib.XEvent()
            fwd_ev.type = xlib.ClientMessage
            fwd_ev.xclient.message_type = self.custom_atom
            fwd_ev.xclient.format = 32
            fwd_ev.xclient.data[0] = ev.type
            xlib.XSendEvent(self.replay_dpy, self.replay_win, False, 0, fwd_ev)


    def _event_callback(self, data):
        self.callback(data)
        return False

    def _event_processed(self, data):
        data.symbol = xlib.XKeysymToString(data.keysym)
        if data.string is None:
            data.string = keysym_to_unicode(data.keysym)
        glib.idle_add(self._event_callback, data)


    def _event_modifiers(self, kev, data):
        data.modifiers = modifiers = {}
        modifiers['shift'] = bool(kev.state & xlib.ShiftMask)
        modifiers['caps_lock'] = bool(kev.state & xlib.LockMask)
        modifiers['ctrl'] = bool(kev.state & xlib.ControlMask)
        modifiers['alt'] = bool(kev.state & xlib.Mod1Mask)
        modifiers['num_lock'] = bool(kev.state & xlib.Mod2Mask)
        modifiers['hyper'] = bool(kev.state & xlib.Mod3Mask)
        modifiers['super'] = bool(kev.state & xlib.Mod4Mask)
        modifiers['alt_gr'] = bool(kev.state & xlib.Mod5Mask)


    def _event_keypress(self, kev, data):
        buf = xlib.create_string_buffer(16)
        keysym = xlib.KeySym()
        status = xlib.Status()
        ret = xlib.Xutf8LookupString(self._kbd_replay_xic, kev, buf, len(buf),
                                     xlib.byref(keysym), xlib.byref(status))
        if ret != xlib.NoSymbol:
            if 32 <= keysym.value <= 126:
                # avoid ctrl sequences, just take the character value
                data.string = chr(keysym.value)
            else:
                try:
                    data.string = buf.value.decode('utf-8')
                except UnicodeDecodeError:
                    pass
        data.keysym = keysym.value
        data.status = status.value


    def _event_lookup(self, kev, data):
        # this is mostly for debugging: we do not account for group/level
        data.keysym = xlib.XkbKeycodeToKeysym(kev.display, kev.keycode, 0, 0)


    def start(self):
        self.lock.acquire()
        self._stop = False
        self.error = None
        super(InputListener, self).start()


    def stop(self):
        with self.lock:
            if not self._stop:
                self._stop = True
                xlib.XRecordDisableContext(self.control_dpy, self.record_ctx)


    def _kbd_init(self):
        self._kbd_last_ev = xlib.XEvent()

        if self.kbd_compose:
            style = xlib.XIMPreeditNothing | xlib.XIMStatusNothing
        else:
            style = xlib.XIMPreeditNone | xlib.XIMStatusNone

        # TODO: implement preedit callbacks for on-the-spot composition
        #       (this would fix focus-stealing for the global IM state)
        self._kbd_replay_xim = xlib.XOpenIM(self.replay_dpy, None, None, None)
        if not self._kbd_replay_xim:
            raise Exception("Cannot initialize input method")

        self._kbd_replay_xic = xlib.XCreateIC(self._kbd_replay_xim,
                                              xlib.XNClientWindow, self.replay_win,
                                              xlib.XNInputStyle, style,
                                              None)
        xlib.XSetICFocus(self._kbd_replay_xic)


    def _kbd_del(self):
        xlib.XDestroyIC(self._kbd_replay_xic)
        xlib.XCloseIM(self._kbd_replay_xim)


    def _kbd_process(self, ev):
        if ev.type == xlib.ClientMessage and \
           ev.xclient.message_type == self.custom_atom:
            if ev.xclient.data[0] in [xlib.FocusIn, xlib.FocusOut]:
                # we do not keep track of multiple XICs, just reset
                xic = xlib.Xutf8ResetIC(self._kbd_replay_xic)
                if xic is not None: xlib.XFree(xic)
            return
        elif ev.type in [xlib.KeyPress, xlib.KeyRelease]:
            # fake keyboard event data for XFilterEvent
            ev.xkey.send_event = False
            ev.xkey.window = self.replay_win

        # pass _all_ events to XFilterEvent
        filtered = bool(xlib.XFilterEvent(ev, 0))
        if ev.type == xlib.KeyRelease and \
           phantom_release(self.replay_dpy, ev.xkey):
            return
        if ev.type not in [xlib.KeyPress, xlib.KeyRelease]:
            return

        # generate new keyboard event
        data = KeyData()
        data.filtered = filtered
        data.pressed = (ev.type == xlib.KeyPress)
        data.repeated = (ev.type == self._kbd_last_ev.type and \
                         ev.xkey.state == self._kbd_last_ev.xkey.state and \
                         ev.xkey.keycode == self._kbd_last_ev.xkey.keycode)
        data.mods_mask = ev.xkey.state
        self._event_modifiers(ev.xkey, data)
        if not data.filtered and data.pressed and self.kbd_translate:
            self._event_keypress(ev.xkey, data)
        else:
            self._event_lookup(ev.xkey, data)
        self._event_processed(data)
        self._kbd_last_ev = ev


    def run(self):
        # control connection
        self.control_dpy = xlib.XOpenDisplay(None)
        xlib.XSynchronize(self.control_dpy, True)

        # unmapped replay window
        self.replay_dpy = xlib.XOpenDisplay(None)
        self.custom_atom = xlib.XInternAtom(self.replay_dpy, b"SCREENKEY", False)
        replay_fd = xlib.XConnectionNumber(self.replay_dpy)
        self.replay_win = create_replay_window(self.replay_dpy)

        # bail during initialization errors
        try:
            if self.input_types & InputType.keyboard:
                self._kbd_init()
        except Exception as e:
            self.error = e
            xlib.XCloseDisplay(self.control_dpy)
            xlib.XDestroyWindow(self.replay_dpy, self.replay_win)
            xlib.XCloseDisplay(self.replay_dpy)

            # cheap wakeup() equivalent for compatibility
            glib.idle_add(self._event_callback, None)

            self._stop = True
            self.lock.release()
            return

        # initialize recording context
        ev_ranges = []
        dev_ranges = []
        if self.input_types & InputType.keyboard:
            ev_ranges.append([xlib.FocusIn, xlib.FocusOut])
            dev_ranges.append([xlib.KeyPress, xlib.KeyRelease])
        if self.input_types & InputType.button:
            dev_ranges.append([xlib.ButtonPress, xlib.ButtonRelease])
        if self.input_types & InputType.movement:
            dev_ranges.append([xlib.MotionNotify, xlib.MotionNotify])
        self.record_ctx = record_context(self.control_dpy, ev_ranges, dev_ranges);

        record_dpy = xlib.XOpenDisplay(None)
        record_fd = xlib.XConnectionNumber(record_dpy)
        # we need to keep the record_ref alive(!)
        record_ref = record_enable(record_dpy, self.record_ctx, self._event_received)

        # event loop
        self.lock.release()
        while True:
            with self.lock:
                if self._stop:
                    break

            r_fd = []
            if xlib.XPending(record_dpy):
                r_fd.append(record_fd)
            if xlib.XPending(self.replay_dpy):
                r_fd.append(replay_fd)
            if not r_fd:
                r_fd, _, _ = select.select([record_fd, replay_fd], [], [])
            if not r_fd:
                break

            if record_fd in r_fd:
                xlib.XRecordProcessReplies(record_dpy)
                xlib.XFlush(self.replay_dpy)

            if replay_fd in r_fd:
                ev = xlib.XEvent()
                xlib.XNextEvent(self.replay_dpy, xlib.byref(ev))
                if self.input_types & InputType.keyboard:
                    self._kbd_process(ev)

        # finalize
        self.lock.acquire()

        xlib.XRecordFreeContext(self.control_dpy, self.record_ctx)
        xlib.XCloseDisplay(self.control_dpy)
        xlib.XCloseDisplay(record_dpy)
        del record_ref

        if self.input_types & InputType.keyboard:
            self._kbd_del()

        xlib.XDestroyWindow(self.replay_dpy, self.replay_win)
        xlib.XCloseDisplay(self.replay_dpy)

        self._stop = True
        self.lock.release()

moveBindings = {
		'Up':(1,0,0,0),
		'o':(1,0,0,-1),
		'Left':(0,0,0,1),
		'Right':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'n':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'Down':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'Prior':(1.1,1.1),
		'Next':(.9,.9),
		'q':(1.1,1),
		'a':(.9,1),
		'w':(1,1.1),
		's':(1,.9),
	      }



def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def callback(data):
	global x
	global y
	global z
	global th
	global status
	global speed
	global turn
	global pub
	global isStop
	
	if isStop == False:
		pressed = False
		repeated = False
		key = ''
	       	values = []
	       	for k in dir(data):
			if k[0] == '_': continue
			if k == 'pressed':
				pressed = True
			if k == 'repeated':
				repeated = True
			if k == 'symbol':
				key = str(getattr(data, k))


		if pressed and repeated and len(key) > 0:
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]


				if speed > 1:
					speed = 1
				if speed < 0:
					speed = 0
				if turn > 1:
					turn = 1
				if turn < 0:
					turn = 0
				print ('speed: ' + str(speed) + '  turn: ' + str(turn))

			else:
				x = 0
				y = 0
				z = 0
				th = 0


			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn

			pub.publish(twist)


def teleop_callback(data):
	global isStop
	cmd = data.data
	print('teleop ' + cmd)
	if cmd == 'start':
		isStop = False
	if cmd == 'stop':
		isStop = True

speed = 0
turn = 0
cmd_topic = 'cmd_vel'
pub = None
sub = None
x = 0
y = 0
z = 0
th = 0
status = 0
isStop = True

if __name__=="__main__":

	speed = 0.5
	turn = 0.5
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	rospy.init_node('mrl_teleop_twist_keyboard')
	cmd_topic = rospy.get_param("~robot_name")

	pub = rospy.Publisher(cmd_topic+'/'+'cmd_vel', Twist, queue_size = 1)
	sub = rospy.Subscriber(cmd_topic+'/'+'teleop',String,teleop_callback)



	glib.threads_init()
	kl = InputListener(callback)
    	try:
        	# keep running only while the listener is alive
        	kl.start()
        	while kl.is_alive():
            		glib.main_context_default().iteration()
    	except KeyboardInterrupt:
        	pass

    	# check if the thread terminated unexpectedly
    	if kl.is_alive():
        	kl.stop()
        	kl.join()
    	elif kl.error:
        	print("initialization error: {}".format(kl.error))
        	if '__traceback__' in dir(kl.error):
            		import traceback
            		traceback.print_tb(kl.error.__traceback__)
        	exit(1)


