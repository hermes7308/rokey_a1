document.addEventListener("DOMContentLoaded", () => {
  // ------------------------- 공통 부분 시작 --------------------------
  const formatTimestamp = (timestampMs) => {
    const dateObject = new Date(timestampMs);

    // 숫자를 두 자릿수로 만들기 위한 헬퍼 함수
    const pad2 = (num) => String(num).padStart(2, "0");

    const year = dateObject.getFullYear();
    // getMonth()는 0부터 시작하므로 1을 더해야 합니다.
    const month = pad2(dateObject.getMonth() + 1);
    const day = pad2(dateObject.getDate());

    const hours = pad2(dateObject.getHours());
    const minutes = pad2(dateObject.getMinutes());
    const seconds = pad2(dateObject.getSeconds());

    // YYYY-MM-DD hh:mm:ss 형태로 조합
    return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
  };
  // ------------------------- 공통 부분 종료 --------------------------

  // ------------------------- UI event 부분 시작 --------------------------
  // Range Slider 값 표시 기능
  const sliders = document.querySelectorAll(".slider");
  sliders.forEach((slider) => {
    const valueSpan = document.getElementById(slider.id + "Value");
    slider.oninput = function () {
      valueSpan.innerHTML = this.value;
    };
  });

  // COPY 버튼 기능
  const copyButtons = document.querySelectorAll(".copy-button");
  const getFormData = (e) => {
    const panel = e.target.closest(".panel");
    const panelTitle = panel.querySelector("h2").textContent;

    let values = {};
    // Number inputs
    panel.querySelectorAll('input[type="number"]').forEach((input) => {
      const labelElement = input.previousElementSibling;
      const label = labelElement.textContent
        .replace(":", "")
        .replace("(deg)", "")
        .replace("(mm)", "")
        .trim();
      values[label] = Number(input.value);
    });

    // Range sliders
    panel.querySelectorAll('input[type="range"]').forEach((slider) => {
      const labelElement = slider.previousElementSibling;
      const label = labelElement.textContent.replace(":", "").trim();
      values[label] = Number(slider.value);
    });
    return { panelTitle, values };
  };
  copyButtons.forEach((button) => {
    button.addEventListener("click", (e) => {
      const { panelTitle, values } = getFormData(e);

      console.log(`[${panelTitle}] Value copy attempt:`, values);
      navigator.clipboard.writeText(JSON.stringify(values));
    });
  });

  const executeAction = (actionType, data) => {
    db.ref("dsr_gss/control_event/action/status")
      .get()
      .then((snapshot) => {
        const currentActionStatus = snapshot.val();
        if (currentActionStatus && currentActionStatus != "DONE") {
          alert("현재 실행중인 작업이 있습니다.");
          return;
        }

        db.ref("dsr_gss/control_event/action").set({
          actionType: actionType,
          data: data,
          status: "READY",
          timestamp: (timestamp = Date.now()),
        });
        alert("새로운 작업이 등록되었습니다.");
      });
    return;
  };

  const getActionButtons = () => {
    const moveButtons = document.querySelectorAll(".move-button");
    const homeButton = [...document.querySelectorAll(".status-button")].filter(
      (btn) => btn.innerHTML.toLowerCase().includes("home")
    )[0];
    return [...moveButtons, homeButton];
  };

  const activateActionButtons = () => {
    getActionButtons().forEach((button) => {
      button.disabled = false;
    });
  };

  const deactivateActionButtons = () => {
    getActionButtons().forEach((button) => {
      button.disabled = true;
    });
  };

  // MOVE 버튼 클릭 시 이벤트 (예시)
  document.querySelectorAll(".move-button").forEach((button) => {
    button.addEventListener("click", (e) => {
      const { panelTitle, values } = getFormData(e);

      let actionType = panelTitle.toLowerCase().includes("movel")
        ? "movel"
        : "movej";

      executeAction(actionType, values);
    });
  });

  // Status 버튼 클릭 시 이벤트 (예시)
  document.querySelectorAll(".status-button").forEach((button) => {
    button.addEventListener("click", (e) => {
      // Home
      if (e.target.textContent.toLowerCase().includes("home")) {
        executeAction("movej", {
          J1: 0.0,
          J2: 0.0,
          J3: 90,
          J4: 0,
          J5: 90,
          J6: 0,
          Velocity: 60,
          Acceleration: 60,
        });
        return;
      }
      // 현재 좌표로 초기화
      if (e.target.textContent.toLowerCase().includes("현재 좌표로 초기화")) {
        if (joint_coordinates != null) {
          let idx = 0;
          document
            .querySelector(".joint-control")
            .querySelectorAll("input[type=number]")
            .forEach(
              (inputElement) => (inputElement.value = joint_coordinates[idx++])
            );
        }
        if (task_coordinates != null) {
          let idx = 0;
          document
            .querySelector(".cartesian-control")
            .querySelectorAll("input[type=number]")
            .forEach(
              (inputElement) => (inputElement.value = task_coordinates[idx++])
            );
        }
        return;
      }
      // 시작
      if (e.target.textContent.toLowerCase().includes("시작")) {
        console.log("시작");
        deactivateActionButtons();
        db.ref("dsr_gss/control_event/required_status").set("RUNNING");
        return;
      }
      // 종료
      if (e.target.textContent.toLowerCase().includes("종료")) {
        console.log("종료");
        activateActionButtons();
        db.ref("dsr_gss/control_event/required_status").set("STOPPED");
        return;
      }
    });
  });

  // Clear Log Button
  const clearLogBtn = document.getElementById("clear-log-btn");
  clearLogBtn.addEventListener("click", () => {
    db.ref("dsr_gss/logs")
      .set("")
      .then(() => console.log("로그 초기화"));
  });

  // ------------------------- UI event 부분 종료 --------------------------

  // ------------------------- 외부 연결 부분 시작 --------------------------
  // PWA
  if ("serviceWorker" in navigator) {
    window.addEventListener("load", () => {
      navigator.serviceWorker
        .register("/sw.js")
        .then(() => console.log("SW registered"))
        .catch((err) => console.error("SW fail", err));
    });
  }

  // PWA Installation Logic
  let deferredPrompt;
  const installButton = document.getElementById("install-pwa-btn");

  window.addEventListener("beforeinstallprompt", (e) => {
    // Prevent the mini-infobar from appearing on mobile
    e.preventDefault();
    // Stash the event so it can be triggered later.
    deferredPrompt = e;
    // Update UI to notify the user they can install the PWA
    installButton.style.display = "block";
  });

  installButton.addEventListener("click", (e) => {
    // hide our user interface that shows our A2HS button
    installButton.style.display = "none";
    // Show the prompt
    deferredPrompt.prompt();
    // Wait for the user to respond to the prompt
    deferredPrompt.userChoice.then((choiceResult) => {
      if (choiceResult.outcome === "accepted") {
        console.log("User accepted the A2HS prompt");
      } else {
        console.log("User dismissed the A2HS prompt");
      }
      deferredPrompt = null;
    });
  });

  window.addEventListener("appinstalled", (evt) => {
    // Log install to analytics
    console.log("INSTALL: Success");
  });

  // Firebase
  const firebaseConfig = {
    apiKey: "AIzaSyCLxzHrMxTnEFLeqJ78f_3cRll1Vbxm6Yg",
    authDomain: "ros2-monitor.firebaseapp.com",
    databaseURL: "https://ros2-monitor-default-rtdb.firebaseio.com",
    projectId: "ros2-monitor",
    storageBucket: "ros2-monitor.firebasestorage.app",
    messagingSenderId: "76402393723",
    appId: "1:76402393723:web:4ba54806c364836145d8de",
  };
  firebase.initializeApp(firebaseConfig);

  // logs
  const db = firebase.database();

  db.ref("dsr_gss/logs").on("value", (snapshot) => {
    const logs = snapshot.val(); // DB에서 값 가져오기
    let textContext = "";

    Object.keys(logs).forEach(
      (key) =>
        (textContext += `[${formatTimestamp(logs[key].timestamp)}][${
          logs[key].level
        }] - ${logs[key].message}\n`)
    );
    document.getElementById("robot-log").innerText = textContext;
  });

  // current_coordinates
  let joint_coordinates = null;
  let task_coordinates = null;
  db.ref("dsr_gss/joint_coordinate/data").on("value", (snapshot) => {
    const coordinates = snapshot.val();
    joint_coordinates = coordinates;
    console.log(joint_coordinates);
  });
  db.ref("dsr_gss/task_coordinate/data").on("value", (snapshot) => {
    const coordinates = snapshot.val();
    task_coordinates = coordinates;
    console.log(task_coordinates);
  });

  // current_status
  db.ref("dsr_gss/control_event/current_status").on("value", (snapshot) => {
    const status = snapshot.val();
    document.querySelector(
      "#current-robot-status"
    ).innerText = `${status.toLowerCase()}`;
  });
  db.ref("dsr_gss/control_event/action").on("value", (snapshot) => {
    const action = snapshot.val();
    if (action.status == "DONE") {
      document.querySelector("#waiting-move-action").innerText = "None";
      return;
    }

    const description =
      action.actionType == "movej"
        ? `J1: ${action.data["J1"]}, J2: ${action.data["J2"]}, J3: ${action.data["J3"]}, J4: ${action.data["J4"]}, J5: ${action.data["J5"]}, J6: ${action.data["J6"]}`
        : `X: ${action.data["X"]}, Y: ${action.data["Y"]}, Z: ${action.data["Z"]}, A: ${action.data["A"]}, B: ${action.data["B"]}, C: ${action.data["C"]}`;

    document.querySelector(
      "#waiting-move-action"
    ).innerText = `[${action.status.toUpperCase()}] - ${
      action.actionType
    } -  ${description}`;
  });
  // ------------------------- 외부 연결 부분 종료 --------------------------
});
