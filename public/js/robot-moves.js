
if (userRole === 1) {
    var movementManager = new ROSLIB.Topic({
        ros: rosBridge,
        name: movementTopic,
        messageType: 'geometry_msgs/Twist'
    });

    socket.on("robot_movement_command", function (data) {
        if (data.roomName !== roomName) {
            return;
        }
        let lX, lY, lZ, aX, aY, aZ;
        if (data.type == "move-forward") {
            lX = parseFloat(data.entity);
            lY = lZ = aX = aY = aZ = 0;
        } else if (data.type == "move-back") {
            lX = -parseFloat(data.entity);
            lY = lZ = aX = aY = aZ = 0;
        } else if (data.type == "move-left") {
            aZ = parseFloat(data.entity);
            lX = lY = lZ = aX = aY = 0;
        } else if (data.type == "move-right") {
            aZ = -parseFloat(data.entity);
            lX = lY = lZ = aX = aY = 0;
        } else {
            lX = lY = lZ = aX = aY = aZ = 0;
        }
        var move = new ROSLIB.Message({
            linear: {
                x: lX,
                y: lY,
                z: lZ
            },
            angular: {
                x: aX,
                y: aY,
                z: aZ
            }
        });
        movementManager.publish(move);
    });
}

let robotMoving = false;

$("#move-forward, #move-back, #move-left, #move-right").click(function() {
    // Prepare message
    let commandName = $(this).attr("id");
    if (robotMoving) {
        commandName = "stop";
    }
    // valoreReale : maxVal = valSlider : 100
    let entity = ( parseFloat($("#maxSpeed").val()) * parseFloat($("#speedVal").val()) ) / 100
    let message = {
        roomName: roomName,
        fromSocket: socket.id,
        fromUsername: userName,
        type: commandName,
        entity: entity,
        timestamp: Date.now()
    };
    console.log(message);
    socket.emit("robot_movement_command", message);
    robotMoving = !robotMoving;
    resetButtons();
    if (robotMoving) {
        $(this).removeClass("btn-info");
        $(this).addClass("btn-danger");
        $(this).html("<i class=\"bi bi-stop-fill\"></i><br>Stop");
    }
});

function resetButtons() {
    $("#move-forward, #move-back, #move-left, #move-right").addClass("btn-info");
    $("#move-forward, #move-back, #move-left, #move-right").removeClass("btn-danger");

    $("#move-forward").html("<i class=\"bi bi-arrow-up\"></i><br>Avanti");
    $("#move-back").html("<i class=\"bi bi-arrow-down\"></i><br>Indietro");
    $("#move-left").html("<i class=\"bi bi-arrow-left\"></i><br>Sinistra");
    $("#move-right").html("<i class=\"bi bi-arrow-right\"></i><br>Destra");
}
