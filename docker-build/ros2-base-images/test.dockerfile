FROM ubuntu/nginx:1.18-22.04_beta
RUN apt-get update && apt-get install -y libc-bin  