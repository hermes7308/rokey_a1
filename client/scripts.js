document.addEventListener('DOMContentLoaded', () => {
    const copyButtons = document.querySelectorAll('.copy-button');

    copyButtons.forEach(button => {
        button.addEventListener('click', (e) => {
            const panel = e.target.closest('.panel');
            const panelTitle = panel.querySelector('h2').textContent;
            const inputs = panel.querySelectorAll('input[type="number"]');
            
            let values = {};
            inputs.forEach(input => {
                const label = input.previousElementSibling.textContent.replace(':', '').trim();
                values[label] = input.value;
            });

            console.log(`[${panelTitle}] 값 복사 시도:`, values);
            alert(`'${panelTitle}'의 현재 값이 복사(시뮬레이션)되었습니다. 콘솔을 확인하세요.`);
        });
    });
    
    // MOVE 버튼 클릭 시 이벤트 (예시)
    document.querySelectorAll('.control-button').forEach(button => {
        if (!button.classList.contains('copy-button')) {
            button.addEventListener('click', (e) => {
                console.log(`[${e.target.textContent}] 명령 전송 시도`);
            });
        }
    });
});