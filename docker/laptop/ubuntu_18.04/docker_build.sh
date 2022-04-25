#!/bin/sh

docker build --rm --build-arg ssh_prv_key="$(cat ~/.ssh/id_ed25519)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" -t diabolokiat/kiat-thesis:laptop-ubuntu18.04 .