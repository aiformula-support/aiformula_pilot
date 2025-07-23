import { getJsonParams } from "./rostopic-viewer.js";

function getSessionInfo(jsonData) {
	const currentTime = new Date();
	const hourRealtime = currentTime.getHours().toString().padStart(2, "0");
	const minutesRealtime = currentTime.getMinutes().toString().padStart(2, "0");
	const secondsRealtime = currentTime.getSeconds().toString().padStart(2, "0");
	const textRealtime =
		hourRealtime + ":" + minutesRealtime + ":" + secondsRealtime;
	document.getElementById("real-time").innerHTML = textRealtime;

	const jsonSessionsList = jsonData.lists;
	let currentSessionIndex = 0;
	for (let i = 0; i < jsonSessionsList.length; i++) {
		const item = jsonSessionsList[i];
		const startTimestamp = Date.parse(item.start);

		if (currentTime > startTimestamp) {
			const endTimestamp = Date.parse(item.end);
			if (currentTime <= endTimestamp) {
				currentSessionIndex = i;
				break;
			}
		}
	}

	const goalTime = new Date(jsonSessionsList[currentSessionIndex].end);

	const countdownTime = goalTime - currentTime;
	const countdownTimer = new Date(countdownTime);
	const minutesCountdown = countdownTimer
		.getMinutes()
		.toString()
		.padStart(2, "0");
	const secondsCountdown = countdownTimer
		.getSeconds()
		.toString()
		.padStart(2, "0");
	const textCountdown =
		minutesCountdown + ":" + secondsCountdown.toString().padStart(2, "0");
	// Uncomment if session time is over 1 hour
	// const hourCoutdown = countdownTimer.getUTCHours().toString().padStart(2, '0');
	// const textCountdown = hourCoutdown + ':' +minutesCountdown +  ':' + secondsCountdown.toString().padStart(2, '0');
	if (countdownTime > 0) {
		document.getElementById("countdown").innerHTML = textCountdown;
	} else {
		document.getElementById("countdown").innerHTML = "00:00";
	}
	document.getElementById("team").innerHTML =
		jsonSessionsList[currentSessionIndex].team;
	document.getElementById("session").innerHTML =
		jsonSessionsList[currentSessionIndex].session;
}

async function timeViewer() {
	const requestPath = "json/session.json";
	const sessionList = await getJsonParams(requestPath);
	setInterval(function () {
		getSessionInfo(sessionList);
	}, 1000);
}

timeViewer();
