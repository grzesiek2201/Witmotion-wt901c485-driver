FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    build-essential \
    libserial-dev \
    g++ \
    cmake \
    && rm -rf /var/lib/apt/lists*


WORKDIR /app
COPY . .

RUN ./complie.sh

# CMD [ "./build/wt901c485_cpp" ]
CMD [ "/bin/bash " ]