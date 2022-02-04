# For better backword compatibility...
FROM ubuntu:16.04
RUN apt update 
RUN apt install -y make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl git sudo

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

WORKDIR /home/user
USER user
CMD /bin/bash


ENV HOME /home/user
ENV PYENV_ROOT $HOME/.pyenv
ENV PATH $PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH
RUN \
  curl https://pyenv.run|bash && \
  echo 'eval "$(pyenv init -)"' >> ~/.bashrc
COPY python_versions.txt $HOME/
RUN  cat python_versions.txt | while read version; do pyenv install $version ; done
RUN  cat python_versions.txt | while read version; do pyenv global $version && pip install scikit-build numpy; done

# Build 
RUN sudo apt install libeigen3-dev cmake -y
RUN git clone https://github.com/HiroIshida/tinyfk.git
RUN cd tinyfk && git submodule update --init 
RUN cd tinyfk && cat ../python_versions.txt | while read version; do pyenv global $version && python setup.py bdist_wheel -p manylinux1_x86_64; done

COPY check_glibcxx.sh $HOME/
RUN bash ./check_glibcxx.sh

RUN \
  pyenv global $(head -n1 python_versions.txt) && \
  pip install twine
