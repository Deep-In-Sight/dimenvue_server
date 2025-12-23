#!/bin/bash

NO_CACHE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --no-cache)
      NO_CACHE="--no-cache"
      shift
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--no-cache]"
      exit 1
      ;;
  esac
done

docker build --network=host $NO_CACHE -f Dockerfile.jetson -t dimenvue_server:jetson .
