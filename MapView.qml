
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtLocation 5.15
import QtPositioning 5.15

Rectangle {
    width: 800
    height: 600

    Plugin {
        id: osmPlugin
        name: "osm"
        PluginParameter {
            name: "osm.mapping.host"
            value: "tile.openstreetmap.org"
        }
        PluginParameter {
            name: "osm.mapping.url"
            value: "/{z}/{x}/{y}.png"
        }
    }

    Map {
        id:map
        anchors.fill: parent
        plugin: osmPlugin
        center: QtPositioning.coordinate(30.9885947, 121.3612901) // 设置初始中心坐标（例如，湖南湘湖坐标  30.1250, 120.2029）上海闵行水池 (30.9885947, 121.3612901)
        zoomLevel: 10
        property var carCoordinate: QtPositioning.coordinate(30.9885947, 121.3612901) // 小车初始位置

        property real carHeading: 0 // 小车初始朝向

        // 添加一个列表来存储标记
        property var markers: []
        // 添加一个列表来存储路径
        property var paths: []
        //添加一个列表来存储小船轨迹
        property var track:[]


        MapPolyline {
            id: routeLine
            line.width: 2
            line.color: 'red'
            path: map.paths.length > 0 ? map.paths[0] : []
        }
        MapPolyline {
            id: trackPolyline
            line.width: 2
            line.color: "yellow"
            path: map.track
        }


        Component.onCompleted: {
            console.log("Map component loaded");
            //updatePositionFromGPS(carCoordinate.latitude, carCoordinate.longitude);
        }
        MouseArea {
            id:mouseArea
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton// | Qt.RightButton
            property real startX: 0
            property real startY: 0
            property var startCenter: QtPositioning.coordinate
            // 鼠标滚轮缩放
            onWheel: {
                handleWheel(wheel)
            }
            function handleWheel(event) {
                if (event.angleDelta.y > 0) {
                    map.zoomLevel += 1
                } else {
                    map.zoomLevel -= 1
                }
            }
            // 鼠标拖动开始
            onPressed: {
                handlePressed(mouse)
            }
            function handlePressed(event) {
                console.log("Pressed at:", event.x, event.y)
                mouseArea.startX = event.x
                mouseArea.startY = event.y
                mouseArea.startCenter = map.center
            }

            // 鼠标位置变化
            onPositionChanged: {
                handlePositionChanged(mouse)
            }

            function handlePositionChanged(event) {
                //console.log("Position changed to:", event.x, event.y)
                var deltaX = mouseArea.startX - event.x
                var deltaY = mouseArea.startY - event.y

                var degreesPerPixelX = 360.0 / (256 * Math.pow(2, map.zoomLevel))
                var degreesPerPixelY = degreesPerPixelX

                var newCenterLat = mouseArea.startCenter.latitude + deltaY * degreesPerPixelY
                var newCenterLon = mouseArea.startCenter.longitude - deltaX * degreesPerPixelX

                map.center.latitude = newCenterLat
                map.center.longitude = newCenterLon
            }
        }
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.RightButton
            onClicked: {
                if (mouse.button === Qt.RightButton) {
                    var coordinate = map.toCoordinate(Qt.point(mouse.x, mouse.y));
                    console.log("Right click at: " + coordinate.latitude + ", " + coordinate.longitude);
                    mapView.handleRightClick(coordinate.latitude, coordinate.longitude);
                }
            }
        }

        MapQuickItem {
            //coordinate: QtPositioning.coordinate(30.1250, 120.2029) // 目标点坐标
            id:car
            coordinate:map.carCoordinate
            anchorPoint.x: image.width / 2
            anchorPoint.y: image.height / 2

            sourceItem: Image {
                id:image
                source: "qrc:/bot.png" // 自定义标记图标
                width: 16
                height: 16
                transform: Rotation {
                    id: carRotation
                    origin.x: image.width / 2
                    origin.y: image.height / 2
                    angle: map.carHeading
                }
            }
            width: image.width
            height: image.height
        }
    }
    Button {
        text: "Calculate Curvature"
        onClicked: mainWindow.onCalculateCurvatureButtonClicked()
    }

    function updateCarPosition(latitude, longitude,heading) {
        //console.log("Updating car position to: ", latitude, longitude)
        map.carCoordinate = QtPositioning.coordinate(latitude, longitude)
        map.carHeading = heading
    }
    function handleRightClick(lat, lon) {
            // 处理右击坐标的函数，可以在此处添加更多逻辑
            console.log("Latitude: " + lat + ", Longitude: " + lon);
    }
    function addMarker(latitude, longitude) {
        if (map.markers.length === 0) {
            // 如果这是第一个标记点，将小车的当前位置添加到路径中
            map.markers.push(map.carCoordinate);
        }
        map.markers.push(QtPositioning.coordinate(latitude, longitude));

        // 更新路径
        if (map.markers.length > 1) {
            var path = [];
            for (var i = 0; i < map.markers.length; i++) {
                path.push(map.markers[i]);
            }
            map.paths = [path];
            updateDistanceLabels(path);
            //console.log("map markers", map.markers);
            //console.log("map paths", map.paths);
        }
        }
    function clearPathsAndMarkers() {
        map.markers = [];
        map.paths = [];
        distanceLabelsModel.clear();
    }

    function calculateDistance(coord1, coord2) {
        var R = 6371e3; // metres
        var lat1 = coord1.latitude * Math.PI/180;
        var lat2 = coord2.latitude * Math.PI/180;
        var deltaLat = (coord2.latitude-coord1.latitude) * Math.PI/180;
        var deltaLon = (coord2.longitude-coord1.longitude) * Math.PI/180;

        var a = Math.sin(deltaLat/2) * Math.sin(deltaLat/2) +
                Math.cos(lat1) * Math.cos(lat2) *
                Math.sin(deltaLon/2) * Math.sin(deltaLon/2);
        var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

        var distance = R * c;
        return distance;
    }

    function updateDistanceLabels(path) {
        distanceLabelsModel.clear();
        for (var i = 1; i < path.length; i++) {
            var midpoint = QtPositioning.coordinate(
                (path[i-1].latitude + path[i].latitude) / 2,
                (path[i-1].longitude + path[i].longitude) / 2
            );
            var distance = calculateDistance(path[i-1], path[i]);
            distanceLabelsModel.append({ coordinate: midpoint, distance: distance.toFixed(2) + " m" });
        }
    }
    function updateTrack(trackCoordinates) {
        map.track = trackCoordinates;
    }

    ListModel {
        id: distanceLabelsModel
    }

    Repeater {
        model: distanceLabelsModel
        MapQuickItem {
            coordinate: model.coordinate
            sourceItem: Rectangle {
                width: 50
                height: 20
                color: "red"
                border.color: "black"
                Text {
                    anchors.centerIn: parent
                    text: model.distance
                    font.pointSize: 10
                }
            }
        }
    }
    Repeater {
        model: map.markers
        MapQuickItem {
            coordinate: modelData
            anchorPoint.x: image.width / 2
            anchorPoint.y: image.height / 2
            sourceItem: Rectangle {
                width: 10
                height: 10
                color: 'red'
                border.color: 'black'
                border.width: 1
                radius: 5
            }
        }
    }
    function getTrack() {
        var trackData = [];
        for (var i = 0; i < map.track.length; i++) {
            trackData.push({ latitude: map.track[i].latitude, longitude: map.track[i].longitude });
        }
        return trackData;
    }

    function printTrack() {
        for (var i = 0; i < map.track.length; i++) {
            console.log("Track Point", i, ":", map.track[i].latitude, map.track[i].longitude);
        }
    }

    function addTrackPoint(latitude, longitude) {
        map.track.push(QtPositioning.coordinate(latitude, longitude));
        console.log("Added Track Point:", latitude, longitude);
    }



}


