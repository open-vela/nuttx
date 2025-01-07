FROM alpine:latest

RUN apk update && apk add --no-cache \
    python3 \
    py3-elftools \
    py3-pip \
    gdb-multiarch \
    build-base \
    && rm -rf /var/cache/apk/*

COPY requirements.txt .

RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

# Clean up the cache and temporary files

RUN rm -rf /var/cache/apk/* \
    && rm -rf /root/.cache

CMD ["/bin/sh"]

# docker build -t pynuttx-alphine .
# docker image save pynuttx-alphine | xz -zc -> pynuttx-alphine.tar.xz
