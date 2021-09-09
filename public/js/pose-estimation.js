
let poseEstimationInProgress = false;
// L'oggetto pose estimator di PoseNet
let poseNetPoseEstimator = null;
// L'oggetto pose estimator di MediaPipe
let mediaPipePoseEstimator = null;

function initializePoseEstimation(el) {
    // Quale algoritmo di pose estimation devo attivare? 1 per MediaPipe, 0 per PoseNet
    let poseEstimationAlg = window.location.search === "?mp" ? 1 : 0;

    if (!poseEstimationAlg) {
        // PoseNet
        posenet.load().then(function (net) {
           poseNetPoseEstimator = net;
           poseEstimationInProgress = true;
        });
    } else {
        // MediaPipe
        mediaPipePoseEstimator = new Holistic({locateFile: (file) => {
            return `https://cdn.jsdelivr.net/npm/@mediapipe/holistic/${file}`;
        }});
        mediaPipePoseEstimator.setOptions({
            modelComplexity: 1,
            smoothLandmarks: true,
            minDetectionConfidence: 0.5,
            minTrackingConfidence: 0.5
        });
        mediaPipePoseEstimator.onResults(gotResults);
        poseEstimationInProgress = true;
    }
}

async function estimatePose(el) {
    // Attenzione! Non chiamare estimatePose prima che poseNetPoseEstimator o mediaPipePoseEstimator siano inizializzati!
    let poseEstimationAlg = window.location.search === "?mp" ? 1 : 0;
    if (!poseEstimationAlg) {
        // PoseNet
        if (!poseEstimationInProgress) {
            return;
        }
        const poses = await poseNetPoseEstimator.estimateMultiplePoses(el, {
            flipHorizontal: false,
            maxDetections: 2,
            scoreThreshold: 0.6,
            nmsRadius: 20});
        gotResults(poses);
    } else {
        // MediaPipe
        await mediaPipePoseEstimator.send({image: el});
    }
}

function gotResults(results) {
    // Riceve i risultati di una pose estimation.
    let poseEstimationAlg = window.location.search === "?mp" ? 1 : 0;
    if (!poseEstimationAlg) {
        // PoseNet
    } else {
        // MediaPipe
    }
    console.log(results);
}