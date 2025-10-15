::Download the dataset from the release section and reconstruct the zip files

mkdir -p data && cd data

curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/autonomous_zipchunk01 > autonomous_zipchunk01
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/autonomous_zipchunk02 > autonomous_zipchunk02
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/autonomous_zipchunk03 > autonomous_zipchunk03
copy /B autonomous_zipchunk* autonomous.zip

curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk01 > piloted_zipchunk01
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk02 > piloted_zipchunk02
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk03 > piloted_zipchunk03
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk04 > piloted_zipchunk04
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk05 > piloted_zipchunk05
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk06 > piloted_zipchunk06
curl https://github.com/Drone-Racing/drone-racing-dataset/releases/download/v3.0.0/piloted_zipchunk07 > piloted_zipchunk07
copy /B piloted_zipchunk* piloted.zip

:: Unzip the downloaded files and then unzip the images and labels of each flight

tar -xf autonomous.zip
tar -xf piloted.zip
cd autonomous
for /d %%r in (*) do for %%s in ("%%r\*.zip") do tar -xf %%s -C %%~dps
cd ../piloted
for /d %%r in (*) do for %%s in ("%%r\*.zip") do tar -xf %%s -C %%~dps