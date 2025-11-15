document.addEventListener("DOMContentLoaded", () => {
  const API_BASE_URL = "http://localhost:5000/api"; // Local Development
  // const API_BASE_URL = "/api" // Production Deployment

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
          `API Error: ${response.status} - ${errorData.message || response.statusText
          }`
        );
      }
      return response.json();
    } catch (error) {
      console.error("Fetch failed:", error.message);
      throw error;
    }
  }

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

      let actionType = panelTitle.toLowerCase().includes("movel") ? "movel" : "movej";

      const data = {
        actionType: actionType,
        data: values
      }
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
    });
  });
});
