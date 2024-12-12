FROM ollama/ollama

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
	ca-certificates \
	python3-dev \
	git \
	wget \
	sudo \
    vim \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN ln -sv /usr/bin/python3 /usr/bin/python

RUN wget https://bootstrap.pypa.io/get-pip.py && \
	python3 get-pip.py "pip<24.1" && \
	rm get-pip.py


RUN pip install \
langchain \
langchain-community \
langchain-core \
langchainhub \
langsmith \
langchain-ollama

