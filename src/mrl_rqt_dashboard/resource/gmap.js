var map = null;
var markers = {};
var markerIndex = 0;
var mapLoaded = false;
var marker = null;

var polygonQueue = [[]];
function initialize(lng, lat, type, zoom)
{
    var mapOptions = {
        "center": new google.maps.LatLng(lat, lng),
        "mapTypeId": type,
        "zoom": zoom
    };
    map = new google.maps.Map(document.getElementById("map_canvas"),
                              mapOptions);
    google.maps.event.addListener(map, "bounds_changed", function() {
        var bounds = map.getBounds();
        var ne = bounds.getNorthEast();
        var sw = bounds.getSouthWest();
        qMapView.regionDidChangeTo(ne.lat(), sw.lat(), ne.lng(), sw.lng());
    });
    google.maps.event.addListener(map, "center_changed", function() {
        var center = map.getCenter();
        qMapView.centerDidChangeTo(center.lat(), center.lng());
    });
    google.maps.event.addListener(map, "click", function(e) {
        var p = e.latLng();
        qMapView.mouseDidClickAt(p.lat(), p.lng());
    });
    google.maps.event.addListener(map, "dblclick", function(e) {
        var p = e.latLng();
        qMapView.mouseDidDoubleClickAt(p.lat(), p.lng());
    });
    google.maps.event.addListener(map, "rightclick", function(e) {
        var p = e.latLng();
        qMapView.mouseDidRightClickAt(p.lat(), p.lng());
    });
    google.maps.event.addListener(map, "drag", function() {
        qMapView.mouseDragged();
    });
    google.maps.event.addListener(map, "dragstart", function() {
        qMapView.mouseDragStarted();
    });
    google.maps.event.addListener(map, "dragend", function() {
        qMapView.mouseDragEnded();
    });
    google.maps.event.addListener(map, "heading_changed", function() {
        qMapView.headingChanged();
    });
    google.maps.event.addListener(map, "idle", function() {
        qMapView.mapBecameIdle();
    });
    google.maps.event.addListener(map, "maptypeid_changed", function() {
        qMapView.mapTypeDidChangeTo(map.getMapTypeId());
    });
    google.maps.event.addListener(map, "mousemove", function(e) {
        var p = e.latLng;
        qMapView.cursorDidMoveTo(p.lat(), p.lng());
    });
    google.maps.event.addListener(map, "mouseover", function(e) {
        var p = e.latLng;
        qMapView.cursorDidEnterTo(p.lat(), p.lng());
    });
    google.maps.event.addListener(map, "mouseout", function(e) {
        var p = e.latLng;
        qMapView.cursorDidLeaveFrom(p.lat(), p.lng());
    });
    google.maps.event.addListener(map, "tilesloaded", function() {
        qMapView.tilesLoaded();
    });
    google.maps.event.addListener(map, "tilt_changed", function() {
        qMapView.tilesChanged();
    });
    google.maps.event.addListener(map, "zoom_changed", function() {
        qMapView.zoomLevelChanged(map.getZoom());
    });
    mapLoaded = true;
}

function appendMarker(name, latitude, longitude)
{
    var marker = new google.maps.Marker({
        position: new google.maps.LatLng(latitude, longitude),
        map: map,
        title: name,
        animation: google.maps.Animation.DROP
    });

     google.maps.event.addListener(marker, "click", function() {
         var index = -1;
         for (var key in markers)
         {
             if (markers[key] === marker)
             {
                 index = key;
                 break;
             }
         }
         qMapView.onMarkerClicked(index);
     });

    markers[markerIndex] = marker;
    markerIndex++;

    return markerIndex - 1;
}

function getMapBounds()
{
    var bounds = map.getBounds();
    var ne = bounds.getNorthEast();
    var sw = bounds.getSouthWest();
    return {
        "north": ne.lng(),
        "south": sw.lng(),
        "east": ne.lat(),
        "west": sw.lat()
    };
}

function getMapCenter()
{
    var center = map.getCenter();
    return {
        "latitude": center.lat(),
        "longitude": center.lng()
    }
}

function setMapCenter(lat, lng, animated)
{
    var latlng = new google.maps.LatLng(lat, lng);
    if (animated)
        map.panTo(latlng);
    else
        map.setCenter(latlng);
}

function boundsFromCoordinates(north, south, east, west)
{
    var ne = new google.maps.LatLng(north, east);
    var sw = new google.maps.LatLng(south, west);
    return new google.maps.LatLngBounds(sw, ne);
}

function panMapToBounds(north, south, east, west)
{
    map.panToBounds(boundsFromCoordinates(north, south, east, west));
}

function fitMapToBounds(north, south, east, west)
{
    map.fitBounds(boundsFromCoordinates(north, south, east, west));
}


function placeMarker(lat, lon) {

    var position = new google.maps.LatLng(lat, lon);
    if (map == null || mapLoaded == false)
    {
        return;
    }
    if (marker != null)
    {
        marker.setMap(Null);
        marker = null;
    }

    marker = new google.maps.Marker({
        position: position,
        map: map
    });
}

function addPolygonPoint(lat, lon)
{
    polygonQueue.push([lat, lon]);
}

function drawPolygon()
{
    appendMarker("123123", polygonQueue[0][0], polygonQueue[0][1]);

    var latLongPoints;
    for(var i=0; i<polygonQueue.length; i++) {
      appendMarker(i.toString(), polygonQueue[i][0], polygonQueue[i][1]);
      latLongPoints.push({
        lat: polygonQueue[i][0],
        lng: polygonQueue[i][1]
      });
    }
    // Construct the polygon.
    var polygon = new google.maps.Polygon({
      paths: latLongPoints,
      strokeColor: '#FF0000',
      strokeOpacity: 0.8,
      strokeWeight: 2,
      fillColor: '#FFFFFF',
      fillOpacity: 0.8
    });
    latLongPoints.setMap(map);

    //polygonQueue = [[]];
}
var count = 0;
function placePolygons(lat1, lon1)
{
//    var latLongPoints;
//    for(var i=0; i<triangleCoords.length; i++) {
//      latLongPoints.push({
//        lat: triangleCoords[i][0],
//        lng: triangleCoords[i][1]
//      });
//    }

////    appendMarker("123123", lat2, lon2);
////    appendMarker("123123", lat3, lon3);
////    appendMarker("123123", lat4, lon4);

//    latLongPoints.push({
//                           lat1: lat1,
//                           lng1: lon1
//                       });
//    latLongPoints.push({
//                           lat2: lat2,
//                           lng2: lon2
//                       });
//    latLongPoints.push({
//                           lat3: lat3,
//                           lng3: lon3
//                       });
//    latLongPoints.push({
//                           lat4: lat4,
//                           lng4: lon4
//                       });

//    // Construct the polygon.
//    var polygon = new google.maps.Polygon({
//      paths: latLongPoints,
//      strokeColor: '#FF0000',
//      strokeOpacity: 0.8,
//      strokeWeight: 2,
//      fillColor: '#FF0000',
//      fillOpacity: 0.35
//    });
//    latLongPoints.setMap(map);
//    polygon.setMap(map);
//    appendMarker("123123", lat1, lon1);

    count++;
    var markerMap = new google.maps.Marker({
      map: map,
      position: new google.maps.LatLng(lat1, lon1),
      title: count.toString()
    });

    // Add circle overlay and bind to marker
    var circle = new google.maps.Circle({
      map: map,
      radius: 7,    // 10 miles in metres
      fillColor: '#AA0000'
    });
    circle.bindTo('center', markerMap, 'position');
//    appendMarker("123123", lat2, lon2);

}
