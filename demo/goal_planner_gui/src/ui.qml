import QtQuick 2.0
import QtQuick.Controls 1.1

// This QML type is used to switch between the gui section which takes care
// for user interaction and the section which displays videos
StackView {
    id: contentSwitchWindow
    initialItem: main
    width: 500; height: 875
    focus : true
    function switch_to_video_view(){
        contentSwitchWindow.push({item: video_widget, replace: true, immediate: true})
    }

    function switch_to_main_view(){
        contentSwitchWindow.push({item: main, replace: true, immediate: true})
    }

    // Used to display videos within the GUI
    Image {
        id: video_widget
        objectName: "video_widget"

        fillMode: Image.PreserveAspectFit

        function call_image_provider() {
            source = "image://ros_image_provider/" + Math.random();
        }

        cache: false
        //width: 500; height: 500
        //source: "image://ros_image_provider/"

        //    id: textTimer
        //    interval: 16
        //    repeat: true
        //    running: true
        //    triggeredOnStart: true
        //    onTriggered: video_widget.call_provider()
        //}


        //MouseArea {
        //    anchors.fill: parent
        //    onClicked: {
        //        parent.call_provider();
                //parent.source = "image://test/world.png?";
                //video_widget.sourceChanged();
                //video_widget.sourceChanged(video_widget.source);
        //        console.log("Try...");
        //   }
        //}

        Rectangle {
            id: video_fixationpoint
            objectName: "video_fixationpoint"

            width: fixationpoint.width
            height: width
            x : fixationpoint.x
            y : fixationpoint.y
            color: "transparent"
            border.color: fixationpoint.border.color
            border.width: fixationpoint.border.width
            radius: fixationpoint.radius
            //anchors.horizontalCenter: main.horizontalCenter
            //anchors.verticalCenter: main.verticalCenter


            Rectangle {
                id: video_fixationpoint_midpoint
                objectName: "video_fixationpoint_midpoint"

                width: fixationpoint_midpoint.width
                height: width
                color: parent.border.color
                border.color: parent.border.color
                border.width: fixationpoint_midpoint.border.width
                radius: fixationpoint_midpoint.radius
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
           }
        }
    }


    // GUI section used by the proband
    Rectangle {
        id: main
        width: 500; height: 875
		focus: true

        property var actions : {
            "actions": function(){ selector.action_menu() ; },
            "goals": function(){ selector.goal_menu() ; },
            "back": function(){ selector.back() ; },
            "choose": function(){ selector.choose() ; },
            "quit": function(){ selector.quit() ; }
        }

        Rectangle {
            id: statusbar
            objectName: "statusbar"
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: 10
            width: 50
            height: 20

            //color: "white"
            Text { id: label; text: "Status Bar"; anchors.centerIn: parent; horizontalAlignment: Text.Center; font.pixelSize: 15; font.weight: Font.bold }
            function writeSomething() {
                label.text = 'Something'
            }
            function display(display_text) {
                label.text = display_text
            }
        } // statusbar Rectangle


        FocusScope {
            objectName: "goal_ui"
            anchors.top: parent.top
            anchors.bottom: statusbar.top
            anchors.left: parent.left
            anchors.right: parent.right
            visible : false

            Rectangle {
                id: header

                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 10
                color : "white"
                z: 2

                Row {
                    visible: currentGoal != undefined
                    height: parent.height
                    spacing : 10

                    Rectangle {
                        width: childrenRect.width// + 20
                        height: parent.height - 20
                        color: "grey"
                        radius: 5
                        ArgumentList { // only one Argument: the current action/function goal				
                        	model: currentGoal != undefined ? [currentGoal.name] : []
                        	height : parent.height
                            id: ref_text
                            anchors.verticalCenter: parent.verticalCenter
                            font.pixelSize: 30
                            font.weight: Font.Bold
                        }
                    }

                    Repeater {
                        model: currentGoal != undefined ? currentGoal.arguments : []
                		
						delegate: Rectangle {
                            width: childrenRect.width
                            height: parent.height - 20
                            radius: 5
                            color: index == currentGoal.current_param ? "green" : "lightgray"
			    			ArgumentList {
								model: modelData
		    
								anchors.verticalCenter: parent.verticalCenter
								height : parent.height
								font.pixelSize: 30
                                font.weight: Font.Bold
                            }
						}
		    		}
				}

                height: 10
                opacity: 0.0

                states: State {
                    name: "populated"
                    when: (currentGoal != undefined)
                    PropertyChanges{ target:header; height: 110; opacity: 1.0 }
                }

                transitions: Transition {
                    NumberAnimation { properties: "height, opacity"; easing.type: Easing.InOutQuad; duration: 250 }
                }
            } // header Rectangle

            Component {
                id: menuDelegate
                Item {
                    width: list.width
                    height: 100

                    Row {
                        spacing : 10

                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.bottom: parent.bottom
                        anchors.leftMargin: 10

                        ArgumentList {
                            model: argument
                            anchors.verticalCenter: parent.verticalCenter
                            height : 80
                            font.pixelSize: 30
                            font.weight: Font.Bold
                        }
                    }

                    Image {
                        id: symbol
                        visible: back ? false : true
                        anchors.right: parent.right
                        anchors.margins: 10
                        height: 85
                        width: 85
                        fillMode: Image.PreserveAspectFit
                        anchors.verticalCenter: parent.verticalCenter
                        horizontalAlignment: Image.AlignHCenter
                        source:  more ? "images/next.png" : "images/ok.png"
                        opacity: 0.3
                    }

                    function callAction() {
                        if (action != "") {
                            main.actions[action]();
                        }
                        else {
                            list.currentIndex = index
                            selector.select_goal(index)
                        }
                    }

                    MouseArea {
                        anchors.fill: parent
                        onClicked: callAction()
                    }

                    Keys.onReturnPressed: callAction()
                    Keys.onRightPressed: callAction()


                }
            }

            Component {
                id: highlightDelegate
                Rectangle {
                    width: list.width
                    height: 100
                    border.color: "lightsteelblue"
                    border.width: 4
                    color: "lightsteelblue"
                    radius: 5
                }
            }


            Rectangle {
                anchors.top: header.bottom
                anchors.bottom: parent.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.topMargin: 30
                anchors.margins: 10
                anchors.bottomMargin: 90
                ListView {
                    id:list
                    anchors.fill: parent
                    keyNavigationWraps: true
                    highlightFollowsCurrentItem: true
                    focus: true
                    model: selectionsModel
                    highlight: highlightDelegate
                    delegate: menuDelegate
                    onCurrentIndexChanged: selector.display_status(currentIndex)
                    //preferredHighlightBegin: (focuspoint.y - list.parent.y - 20)
                    //preferredHighlightEnd: (focuspoint.y - list.parent.y - 20)
                    preferredHighlightBegin: fixationpoint.midpoint - list.parent.y - 50
                    preferredHighlightEnd: fixationpoint.midpoint - list.parent.y - 50
                    highlightRangeMode: ListView.StrictlyEnforceRange
                    Keys.onLeftPressed: {
                        selector.back()
                        event.accepted = true
                    }

//                    Component.onCompleted: {
//                        for (var prop in list.parent) {
//                            print(prop += " (" + typeof(list.parent[prop]) + ") = " + list.parent[prop]);
//                        }
//                        print("fixationpoint.y: " + fixationpoint.y)
//                        print("list.parent.y: " + list.parent.y);
//                        print("preferedHighlightBegin: " + (fixationpoint.y - list.parent.y - 20));
//                        print("fixationpoint.top: " + fixationpoint.top);
//                    }
                }
            }

            function teleop_up() {
                list.decrementCurrentIndex()
            }
            function teleop_down() {
                list.incrementCurrentIndex()
            }
            function teleop_select() {
                list.currentItem.callAction()
            }
            function teleop_back() {
                selector.back()
            }
        } // "goal_ui" FocusScope

        FocusScope {
            objectName: "planner_ui"
            id: planner_ui
            anchors.top: parent.top
            anchors.bottom: statusbar.top
            anchors.left: parent.left
            anchors.right: parent.right
            visible: false

            Component {
                id: planDelegate
                Item {
                    id : planRect
                    width: planView.width
                    height: 100

                    Row {
                        spacing : 10

                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        anchors.margins: 10

                        Rectangle {
                            anchors.verticalCenter: parent.verticalCenter
                            width: childrenRect.width
                            height: parent.height

                            color: "grey"
                            radius: 5
                            ArgumentList {
                                height: parent.height
                                font.pixelSize: 40
                                font.weight: Font.Bold

                                model : [name]
                            }
                        }

                        Repeater {
                            model : modelData.arguments

                            Rectangle {
                                anchors.verticalCenter: parent.verticalCenter
                                width: childrenRect.width
                                height: parent.height

                                color: "lightgrey"
                                radius: 5

                                ArgumentList {
                                    anchors.verticalCenter: parent.verticalCenter
                                    height: parent.height
                                    font.pixelSize: 30
                                    font.weight: Font.Bold
                                    model: modelData
                                }
                            }
                        }
                    }

                    Image {
                        id: actionSymbol
                        visible: true
                        anchors.right: parent.right
                        anchors.margins: 10
                        height: 85
                        width: 85
                        fillMode: Image.PreserveAspectFit
                        anchors.verticalCenter: parent.verticalCenter
                        horizontalAlignment: Image.AlignHCenter
                        source: action_status == C.ActionStatus.EXECUTABLE ? "images/ok.png" : ""
                        opacity: 0.3
                    }

                    function planActionClick() {
                        list.currentIndex = index
                        selector.select_action(index)
                    }

                    MouseArea {
                        anchors.fill: parent
                        onClicked: planActionClick()
                    }

                    Keys.onReturnPressed: planActionClick()
                    Keys.onRightPressed: planActionClick()
                    Keys.onDownPressed: planner_ui.teleop_down()
                    Keys.onUpPressed: planner_ui.teleop_up()

                    Rectangle {
                        anchors.fill: planRect
                        anchors.margins: 2
                        z: -1
                        radius: 5
                        // border.color: "black"
                        // border.width: 2
                        visible : action_status != C.ActionStatus.EXECUTABLE
                        color: {
                            if (action_status == C.ActionStatus.EXECUTED)
                                return "lightgrey"
                            else if (action_status == C.ActionStatus.FAILED)
                                return "red"
                            else if (action_status == C.ActionStatus.UNSUCCESSFUL)
                                return "red"
                            else if (action_status == C.ActionStatus.IN_PROGRESS)
                                return "green"
                            else return "white"
                        }
                    }
                }
            }

            onActiveFocusChanged: {
                // Move the selection to the first executable action
                var executable = false;
                var new_index = 0
                for (var i = planView.currentIndex; i < planView.count; i++) {
                    var item = planView.model[i]
                    if (item.action_status == C.ActionStatus.EXECUTABLE) {
                        new_index = i;
                        executable = true;
                        break;
                    }
                }
                planView.currentIndex = new_index;

                // if (executable) {
                //     planView.enabled = true;
                // }
                // else {
                //     planView.enabled = true;
                // }
            }

            Component {
                id: planHighlightDelegate
                Rectangle {
                    width: planView.width
                    height: 100
                    border.color: "lightsteelblue"
                    border.width: 4
                    color: "lightsteelblue"
                    radius: 5
                }
            }

            Rectangle {
                anchors.fill: parent
                anchors.margins: 10
                anchors.bottomMargin: 90
                ListView {
                    objectName:"planView"
                    id:planView
                    anchors.fill: parent
                    keyNavigationWraps: true
                    highlightFollowsCurrentItem: true
                    focus: true
                    model: planModel
                    populate: Transition {
                        enabled : false
                    }
                    highlight: planHighlightDelegate
                    delegate: planDelegate
                    onCurrentIndexChanged: selector.display_status(currentIndex)
                    //preferredHighlightBegin: (focuspoint.y - planView.parent.y - 20)
                    //preferredHighlightEnd: (focuspoint.y - planView.parent.y - 20)
                    preferredHighlightBegin: fixationpoint.midpoint - planView.parent.y - 50
                    preferredHighlightEnd: fixationpoint.midpoint - planView.parent.y - 50
                    highlightRangeMode: ListView.StrictlyEnforceRange

                    Keys.onLeftPressed: {
                        selector.back()
                        event.accepted = true
                    }

                    //Component.onCompleted: {
                        //for (var prop in list.parent) {
                        //    print(prop += " (" + typeof(list.parent[prop]) + ") = " + list.parent[prop]);
                        //}
                        //print("fixationpoint.y: " + fixationpoint.y)
                        //print("list.parent.y: " + list.parent.y);
                        //print("preferedHighlightBegin (planView): " + (fixationpoint.y - planView.parent.y - 20));
                        //print("fixationpoint.top: " + fixationpoint.top);
                    //}
                    //property var tmp: planView.parent.y

                    //onTmpChanged: {console.log("The next color will be: ")}
                }
            }
            function teleop_up() {
                if (planView.currentIndex > 0) {
                    var next = planView.model[planView.currentIndex - 1]
                    if (next.action_status == C.ActionStatus.EXECUTABLE) {
                        planView.decrementCurrentIndex()
                    }
                }
            }
            function teleop_down() {
                if (planView.currentIndex < planView.count - 1) {
                    var next = planView.model[planView.currentIndex + 1]
                    if (next.action_status == C.ActionStatus.EXECUTABLE) {
                        planView.incrementCurrentIndex();
                    }
                }
            }
            function teleop_select() {
                planView.currentItem.planActionClick()
            }
            function teleop_back() {
                selector.back()
            }
        }

        FocusScope {
            objectName: "world_image"
            id: worldfocus
            anchors.top: parent.top
            anchors.bottom: statusbar.top
            anchors.left: parent.left
            anchors.right: parent.right
            visible : false
            AnimatedImage {
                id: world_turning
                anchors.fill: parent
                anchors.margins: 10
                source:  "images/rotating_globes.gif"
                opacity: 1
                SequentialAnimation on opacity {
                    id: world_animation
                    NumberAnimation{from:1; to:0; duration:2000
                    }
                    onStopped: worldfocus.visible = false
                }
            }

            function turn_world() {
                visible = true
                world_turning.opacity = 1
                world_animation.start()
            }
        }

        FocusScope {
            objectName: "compute_image_focus"
            id: computeImageFocus
            anchors.top: parent.top
            anchors.bottom: statusbar.top
            anchors.left: parent.left
            anchors.right: parent.right
            visible : false

            Rectangle {
                objectName: "compute_image_back"
                id: compute_image_back
                color: "gray"
                width: main.width
                height: main.height
                opacity: 0

                NumberAnimation on opacity {
                    id: fade_in_animation
                    from: 0
                    to: 0.5
                    duration: 800
                }

                NumberAnimation on opacity {
                    id: fade_out_animation
                    from: 0.5
                    to: 0
                    duration: 800
                }
            }

            AnimatedImage {
                objectName: "compute_image"
                id: computeimage
                anchors.horizontalCenter: parent.horizontalCenter
                width: 120; height: 120
                y: main.y + main.height / 2 - height / 2
    //            visible: true
                fillMode: Image.PreserveAspectFit
                source: "images/loading.gif"
                opacity: 0
            }

            function turnOn() {
                visible = true
                fade_in_animation.start()
                computeimage.opacity = 1
            }

            function turnOff() {
                fade_out_animation.start()
                computeimage.opacity = 0
            }
        }

        Image {
            id: focuspoint
            objectName: "focuspoint"
            visible: true
            //visible: false
            width: 60; height: 60
            y: main.y + main.height / 2 - height / 2
            anchors.horizontalCenter: parent.horizontalCenter
            fillMode: Image.PreserveAspectFit
            source: ""
            opacity: 1
            MouseArea {
                    anchors.fill: parent
                    drag.target: focuspoint
                    drag.axis: Drag.YAxis
                    drag.minimumY: 0
            }
            function set_image(filename) {
                source = filename
            }
        }

        Rectangle {
            id: fixationpoint
            objectName: "fixationpoint"
            property int ring_diameter: 15
            property int midpoint_diameter: 0
            property int ring_thickness: 3
            height: ring_diameter; width: height
            y: main.y + main.height / 2 - height / 2
            color: "transparent"
            border.color: "blue"
            border.width: ring_thickness
            radius: width*0.5
            //anchors.verticalCenter: main.verticalCenter
            anchors.horizontalCenter: main.horizontalCenter
            property int midpoint: y + height / 2

            MouseArea {
                anchors.fill: parent
                drag.target: fixationpoint
                drag.axis: Drag.YAxis
                drag.minimumY: 0
            }

            function change_fixationpoint_color(fixationpoint_color)
            {
                border.color = fixationpoint_color
            }

            function change_fixationpoint_midpoint_diameter(diameter)
            {
                midpoint_diameter = diameter
            }

            function change_fixationpoint_ring_diameter(diameter)
            {
                ring_diameter = diameter
            }

            function change_fixationpoint_ring_thickness(thickness)
            {
                ring_thickness = thickness
            }

            Rectangle {
               id: fixationpoint_midpoint
               objectName: "fixationpoint_midpoint"

               height: parent.midpoint_diameter
               width: height
               color: parent.border.color
               border.color: parent.border.color
               border.width: 1
               radius: width*0.5
               anchors.verticalCenter: parent.verticalCenter
               anchors.horizontalCenter: parent.horizontalCenter
            }
        }
    }

}
