document.addEventListener("DOMContentLoaded", () => {
  // ------------------------- 공통 부분 시작 --------------------------
  // const API_BASE_URL = "http://localhost:5000/api"; // Local Development
  const API_BASE_URL = "/api"; // Production Depleoyment

  async function apiFetch(endpoint, options = {}) {
    const url = `${API_BASE_URL}/${endpoint}`;

    const defaultHeaders = {
      "Content-Type": "application/json",
    };

    const finalOptions = {
      ...options,
      headers: {
        ...defaultHeaders,
        ...options.headers,
      },
    };

    try {
      const response = await fetch(url, finalOptions);
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          `API Error: ${response.status} - ${
            errorData.message || response.statusText
          }`
        );
      }
      return response.json();
    } catch (error) {
      console.error("Fetch failed:", error.message);
      throw error;
    }
  }
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

  // MOVE 버튼 클릭 시 이벤트 (예시)
  document.querySelectorAll(".move-button").forEach((button) => {
    button.addEventListener("click", (e) => {
      const { panelTitle, values } = getFormData(e);

      let actionType = panelTitle.toLowerCase().includes("movel")
        ? "movel"
        : "movej";

      const data = {
        actionType: actionType,
        data: values,
      };
      apiFetch("execute_action", { method: "POST", body: JSON.stringify(data) })
        .then((data) => alert(data.message))
        .catch((err) => console.log(err));
    });
  });

  // Status 버튼 클릭 시 이벤트 (예시)
  document.querySelectorAll(".status-button").forEach((button) => {
    button.addEventListener("click", (e) => {
      console.log(`[${e.target.textContent}] 상태/도구 명령 전송 시도`);
      alert(`'${e.target.textContent}' 명령이 실행(시뮬레이션)되었습니다.`);
      // Home
      if (e.target.textContent.toLowerCase().includes("home")) {
        apiFetch("execute_action", {
          method: "POST",
          body: JSON.stringify({
            actionType: "movej",
            data: {
              J1: 0.0,
              J2: 0.0,
              J3: 90,
              J4: 0,
              J5: 90,
              J6: 0,
              Velocity: 60,
              Acceleration: 60,
            },
          }),
        })
          .then((res) => alert(res.message))
          .catch((err) => console.log(err));
        return;
      }
      // 현재 좌표로 초기화
      if (e.target.textContent.toLowerCase().includes("현재 좌표로 초기화")) {
        apiFetch("get_current_coordinates", { method: "GET" })
          .then((res) => {
            if (res.status === "success") {
              let idx = 0;
              document
                .querySelector(".joint-control")
                .querySelectorAll("input[type=number]")
                .forEach(
                  (inputElement) =>
                    (inputElement.value =
                      res.data.joint_coordinates.data[idx++])
                );
              idx = 0;
              document
                .querySelector(".cartesian-control")
                .querySelectorAll("input[type=number]")
                .forEach(
                  (inputElement) =>
                    (inputElement.value =
                      res.data.joint_coordinates.data[idx++])
                );
              return;
            }
          })
          .catch((err) => console.log(err));
        return;
      }
    });
  });

  // Clear Log Button
  const clearLogBtn = document.getElementById("clear-log-btn");
  clearLogBtn.addEventListener("click", () => {
    apiFetch("clear_logs", { method: "POST" })
      .then((res) => console.log(res))
      .catch((err) => console.log(err));
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
  // Firebase 앱 초기화
  firebase.initializeApp(firebaseConfig);

  // 3. Realtime Database 참조 가져오기
  const db_ref = firebase.database();

  // 2단계 Python 코드에서 'robot_status/completed_jobs' 경로에 저장했음
  const jobCountRef = db_ref.ref("dsr_gss/logs");

  // 4. 데이터 실시간 수신 (핵심!)
  // 'on' 리스너는 DB 데이터가 변경될 때마다 *자동으로* 호출됩니다.
  jobCountRef.on("value", (snapshot) => {
    const logs = snapshot.val(); // DB에서 값 가져오기
    let textContext = "";
    console.log(logs);

    Object.keys(logs).forEach(
      (key) => (textContext += `${logs[key].message}\n`)
    );
    document.getElementById("robot-log").innerText = textContext;

    // 5. HTML 요소 업데이트
    // document.getElementById("job-count").innerText = jobCount || 0;
  });
  // ------------------------- 외부 연결 부분 종료 --------------------------
});
