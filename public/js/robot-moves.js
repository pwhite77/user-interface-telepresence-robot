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
