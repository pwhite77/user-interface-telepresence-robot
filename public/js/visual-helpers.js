function decideChatHeight() {
    $("#chatMessages").height(
        Math.round($( window ).height() * 60 / 100)
    );
}

// Funzione per aspettare la fine del ridimensionamento.
// Taken from: https://stackoverflow.com/a/45905199
function debounce(func) {
    let timer;
    return function(event){
        if(timer) clearTimeout(timer);
        timer = setTimeout(func,100,event);
    };
}

sendButton.addEventListener("click", function() {
    sendMessage("message", "text", $("#message").val());
    $("#message").val("");
});

/*window.addEventListener("resize", debounce(function(e) {
    decideChatHeight();
    if ( !$("body > video").length ) {
        return;
    }
    $("body > video").first().css({
        "width": $(document).width() + "px",
        "height": ($(document).height() - $("#videos").height()) + "px"
    });
}));*/

window.addEventListener("load", function() {
    decideChatHeight();
});

var chatModalAct = new bootstrap.Modal(document.getElementById('chatModal'), {
    keyboard: false
});
document.querySelector("#messageNotification").addEventListener("click", function() {
    chatModalAct.show();
});

let chatModal = document.querySelector("#chatModal");
chatModal.addEventListener("show.bs.modal", function(event) {
    // Questo previene il bug su Android che la lista dei comandi si sovrappone al campo testo della chat.
    $("#controls").hide();
});
chatModal.addEventListener("hide.bs.modal", function(event) {
    // Quando si è finito di chattare, mostra di nuovo i controlli.
    $("#controls").show();
});

document.querySelector("#cameraSwitchModal").addEventListener("show.bs.modal", async function(event) {
    let cameras = await getConnectedVideoDevices();
    $.each(cameras, function (i, item) {
        $('#cameras').append($('<option>', {
            value: item.deviceId,
            text : item.label
        }));
    });
});