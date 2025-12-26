#!/bin/bash

NO_CACHE=""
BUILD_BASE=false
BUILD_APP=true

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --no-cache)
      NO_CACHE="--no-cache"
      shift
      ;;
    --base)
      BUILD_BASE=true
      BUILD_APP=false
      shift
      ;;
    --app)
      BUILD_BASE=false
      BUILD_APP=true
      shift
      ;;  
    --all)
      BUILD_BASE=true
      BUILD_APP=true
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --no-cache  Build without using cache"
      echo "  --base      Build only the base image (dimenvue_server_base:jetson)"
      echo "  --app       Build only the application image (dimenvue_server:jetson)"
      echo "  --all       Build both base and application images"
      echo "  --help      Show this help message"
      echo ""
      echo "By default, only the application image (dimenvue_server:jetson) is built."
      echo "The base image must exist before building the application image."
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

# Build base image if requested
if [ "$BUILD_BASE" = true ]; then
  echo "=== Building base image: dimenvue_server_base:jetson ==="
  docker build --network=host $NO_CACHE -f Dockerfile.jetson.base -t dimenvue_server_base:jetson .
  if [ $? -ne 0 ]; then
    echo "Error: Base image build failed"
    exit 1
  fi
  echo "=== Base image built successfully ==="
fi

# Build application image if requested
if [ "$BUILD_APP" = true ]; then
  echo "=== Building application image: dimenvue_server:jetson ==="
  docker build --network=host $NO_CACHE -f Dockerfile.jetson -t dimenvue_server:jetson .
  if [ $? -ne 0 ]; then
    echo "Error: Application image build failed"
    exit 1
  fi
  echo "=== Application image built successfully ==="
fi
