import QtQuick 2.0

Row {
    id: main
    
    property font font
    property alias model : repeater.model
    property real margin : 5
    
    Repeater {
        id : repeater
        anchors.verticalCenter: parent.verticalCenter
        
        Item {
            width: childrenRect.width + 2*main.margin
            height: parent.height

            Text {
                id: ref_text 
                x: main.margin
                anchors.verticalCenter: parent.verticalCenter
                font : main.font
                text: modelData.image === "" ? modelData.text : ""
            }
            Image {
                id: ref_image
                x: main.margin
                height: parent.height
                fillMode: Image.PreserveAspectFit
                anchors.verticalCenter: parent.verticalCenter
                horizontalAlignment: Image.AlignHCenter
                source: modelData.image
            }
        }
    }
}
