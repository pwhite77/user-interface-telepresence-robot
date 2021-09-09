const totalPages = 3;

$("#makeOrJoinRoom").click(function () {
    roomName = $("#roomName").val();
    roomPassword = $("#roomPassword").val();

    $.post( "api/room", {"roomName": roomName, "roomPassword": roomPassword})
        .done(function( data ) {
            if (data.success) {
                // Imposta i cookie*/
                goToPage(2);
                startLocalVideo();
            } else {
                alert("Si è verificato un errore. Ricontrolla i dati inseriti e riprova.");
            }
        });
});

$("#login").click(function() {
    userName = $("#userName").val();
    $.post("api/login/", {"username": userName, "password": $("#userPassword").val()})
        .done(async function(data) {
            if (data.success) {
                userRole = data.role;
                goToPage(1);
            } else {
                alert("Si è verificato un errore. Ricontrolla i dati inseriti e riprova.");
            }
        });

});

function goToPage(pageNumber) {
    for (i = 0; i < totalPages; i++) {
        document.getElementById("page-" + i).style.display = "none";
    }
    document.getElementById("page-" + pageNumber).style.display = "block";
}