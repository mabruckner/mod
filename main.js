var app = new Vue({
    el: "#app",
    data: {
        message: "hello world"
    }
});

setInterval(() => {
    console.log(navigator.getGamepads());
}, 1000.0);

window.addEventListener("gamepadconnected", function(e) {
  var gp = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    gp.index, gp.id,
    gp.buttons.length, gp.axes.length);
});
