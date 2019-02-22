const SECONDS = 0.25;
const BUFFER_SIZE = SECONDS * 256;
const WEIGHT = 0.95;

let buffer = new Array();
let weighted = {
    alpha: -1,
    beta: -1,
    theta: -1,
    engagement: -1
};

window.Device = new Bluetooth.BCIDevice((sample) => {
    if (Bluetooth.BCIDevice.electrodeIndex("AF7") !== sample.electrode) return;

    sample.data.forEach(el => {
        if (buffer.length > BUFFER_SIZE) buffer.shift();
        buffer.push(el);
    });

    if (buffer.length < BUFFER_SIZE) return;

    let psd = window.bci.signal.getPSD(BUFFER_SIZE, buffer);

    let alpha = window.bci.signal.getBandPower(BUFFER_SIZE, psd, 256, "alpha");
    let beta = window.bci.signal.getBandPower(BUFFER_SIZE, psd, 256, "beta");
    let theta = window.bci.signal.getBandPower(BUFFER_SIZE, psd, 256, "theta");
    let engagement = beta / (alpha + theta);
    let sum = alpha + beta + theta;

    let w_alpha = alpha / sum;
    let w_beta = beta / sum;
    let w_theta = theta / sum;

    if (weighted.alpha < 0) {
        weighted.alpha = w_alpha || 0;
        weighted.beta = w_beta || 0;
        weighted.theta = w_theta || 0;
        weighted.engagement = engagement || 0;
    } else {
        weighted.alpha = weighted.alpha * WEIGHT + (w_alpha || 0) * (1 - WEIGHT);
        weighted.beta = weighted.beta * WEIGHT + (w_beta || 0) * (1 - WEIGHT);
        weighted.theta = weighted.theta * WEIGHT + (w_theta || 0) * (1 - WEIGHT);
        weighted.engagement = weighted.engagement * WEIGHT + (engagement || 0) * (1 - WEIGHT);
    }

    //console.log(weighted.alpha + " : " + weighted.beta + " : " + weighted.theta);
    //console.log(weighted.engagement);

    if (window.gameInstance.__ready == true) {
        //console.log("gameInstance ready", weighted.engagement)
        window.gameInstance.SendMessage("Drone", "SetSpeed", weighted.engagement);
    }
});

let connect = async() => {
    try {
        await window.Device.connect();

        window.gameInstance = UnityLoader.instantiate("gameContainer", "Build/bdr-simulator.json", {onProgress: UnityProgress, Module: {
            onRuntimeInitialized: function () {
              UnityProgress(gameInstance, "complete");
              window.gameInstance.__ready = true;
            },
          }});
    } catch (e) {
        console.log("connect/load error. retrying...");
        connect();
    }
}