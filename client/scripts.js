document.addEventListener('DOMContentLoaded', () => {
    // Range Slider 값 표시 기능
    const sliders = document.querySelectorAll('.slider');
    sliders.forEach(slider => {
        const valueSpan = document.getElementById(slider.id + 'Value');
        slider.oninput = function() {
            valueSpan.innerHTML = this.value;
        };
    });

    // COPY 버튼 기능
    const copyButtons = document.querySelectorAll('.copy-button');
    copyButtons.forEach(button => {
        button.addEventListener('click', (e) => {
            const panel = e.target.closest('.panel');
            const panelTitle = panel.querySelector('h2').textContent;
            
            let values = {};
            // Number inputs
            panel.querySelectorAll('input[type="number"]').forEach(input => {
                const labelElement = input.previousElementSibling;
                // Use data-label if available, otherwise fallback to textContent
                const label = labelElement.dataset.label ? labelElement.dataset.label + " (deg/mm)" : labelElement.textContent.replace(':', '').trim();
                values[label] = input.value;
            });

            // Range sliders
            panel.querySelectorAll('input[type="range"]').forEach(slider => {
                const labelElement = slider.previousElementSibling;
                const label = labelElement.textContent.replace(':', '').trim();
                values[label] = slider.value;
            });

            console.log(`[${panelTitle}] Value copy attempt:`, values);
            alert(`'${panelTitle}'의 현재 설정 값이 복사(시뮬레이션)되었습니다. 콘솔을 확인하세요.`);
        });
    });
    
    // MOVE 버튼 클릭 시 이벤트 (예시)
    document.querySelectorAll('.move-button').forEach(button => {
        button.addEventListener('click', (e) => {
            console.log(`[${e.target.textContent}] 명령 전송 시도`);
            alert(`'${e.target.textContent}' 명령이 전송(시뮬레이션)되었습니다.`);
        });
    });

    // Status 버튼 클릭 시 이벤트 (예시)
    document.querySelectorAll('.status-button').forEach(button => {
        button.addEventListener('click', (e) => {
            console.log(`[${e.target.textContent}] 상태/도구 명령 전송 시도`);
            alert(`'${e.target.textContent}' 명령이 실행(시뮬레이션)되었습니다.`);
        });
    });
});