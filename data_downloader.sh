# Download the dataset from the release section and reconstruct the zip files

mkdir -p data && cd data

wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/autonomous_zipchunk01
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/autonomous_zipchunk02
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/autonomous_zipchunk03
cat autonomous_zipchunk* > autonomous.zip
rm autonomous_zipchunk*

wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/piloted_zipchunk01
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/piloted_zipchunk02
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/piloted_zipchunk03
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/piloted_zipchunk04
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/piloted_zipchunk05
wget https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v2.0.0/piloted_zipchunk06
cat piloted_zipchunk* > piloted.zip
rm piloted_zipchunk*

# Unzip the downloaded files and then unzip the images and labels of each flight:

find . -type f -name '*.zip' -exec unzip {} \;
find autonomous -type f -name '*.zip' -execdir unzip {} \;
find piloted -type f -name '*.zip' -execdir unzip {} \;