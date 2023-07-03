#!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

# Default options
DOCKER=moma_dev
DOCKERFILE=dev.Dockerfile
BUILD=false

help()
{
    echo "Usage: run_docker.sh [ -d | --docker <image name> ]
               [ -b | --build <dockerfile name> ]
               [ -h | --help  ]"
    exit 2
}

SHORT=d:,b:,h
LONG=docker:,build:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -d | --docker )
      DOCKER="$2"
      shift 2
      ;;
    -b | --build )
      BUILD="true"
      DOCKERFILE="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done

if [ "$BUILD" = true ]; then
     docker build -f $DOCKERFILE -t $DOCKER .
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="FRANKA_IP=$FRANKA_IP" \
    --volume=/home/$USER/moma_ws:/root/moma_ws \
    --volume=/home/$USER/data:/root/data \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --name moma \
    ${DOCKER} \
    bash

echo "Done."