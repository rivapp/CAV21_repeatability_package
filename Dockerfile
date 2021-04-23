FROM ubuntu

MAINTAINER Radoslav Ivanov

RUN apt-get -y update
RUN apt install build-essential -y --no-install-recommends

RUN apt install -y libgmp3-dev libmpfr-dev libmpfr-doc libmpfr6 gsl-bin libgsl0-dev bison flex gnuplot-x11 libglpk-dev libyaml-cpp-dev m4 gcc-8 g++-8 libopenmpi-dev python3-pip emacs

RUN pip3 install pip --upgrade

COPY ./ReachNNStar /home/reachNNStar
COPY ./verisig /home/verisig
COPY ./tmp /home/tmp
COPY ./verisig_models /home/verisig_models

RUN cd /home/verisig/flowstar && make
RUN cd /home/tmp && make
RUN cd /home/reachNNStar && pip install -r requirements.txt && ./compile.sh