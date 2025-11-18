# nginx로 정적 서비스
FROM nginx:stable-alpine

COPY nginx.conf /etc/nginx/nginx.conf

# 빌드 결과물 복사 (browser 폴더 내부만)
COPY . /usr/share/nginx/html

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"] 