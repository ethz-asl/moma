set -e
docker build -t mobmi .
docker save -o ../image.zip mobmi

