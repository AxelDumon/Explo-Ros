# Installation procedures

```
# prerequisites
sudo apt update
sudo apt install git-all unzip build-essential npm python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update

sudo timedatectl set-ntp false
sudo timedatectl set-time "2025-12-19 15:30:00"
# remplacer par l'heure et la date actuelle ^
sudo npm i -g n typescript
sudo n lts && npm uninstall n

```

```
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
mkdir  -p  ~/ros2_ws/src
cd  ~/ros2_ws

rosdep install -i --from-path src --rosdistro jazzy -y && colcon build
source ~/ros2_ws/install/setup.bash && grep -qxF 'source ~/ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
cd src
ros2 pkg create --build-type ament_cmake --maintainer-name axel --maintainer-email "axel.dumon.etu@univ-lille.fr" --description "Exploration JS Package interacting with CouchDB" --license Apache-2.0 node_pkg
mkdir -p node_pkg/js
mkdir -p node_pkg/msg
echo "int32 x
int32 y
int8 value" > ./node_pkg/msg/Cell.msg
echo "Cell[] cells" > ./node_pkg/msg/MapUpdate.msg
```

Put inside the CMakeList... & package.xml :
```
# CMakeLists
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cell.msg"
  "msg/MapUpdate.msg"
)

# package.xml
<member_of_group>rosidl_interface_packages</member_of_group>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

```
cd node_pkg/js
wget -O Explo-Ros.zip https://github.com/AxelDumon/Explo-Ros/archive/refs/heads/master.zip && unzip Explo-Ros.zip && mv *master Explo-Ros && rm -Rf Explo-Ros.zip
npm i && npm run build
cd ~/ros2_ws
colcon build --packages-select node_pkg
```

Modifiez ensuite les fichiers *model/db/couchManagerConf.json* ainsu que *model/db/couchHosts.json* pour qu'ils correspondent à votre configuration à vous

Si vous obtenez une erreur par rapport à un fichier "design_docs" qui n'existe pas créez le ainsi : 

```
mkdir build/models/db/views/design_docs
```

N'oubliez pas de changer le nom de votre agent dans la configuration json AINSI que dans app.ts dans l'initialisation de l'agent.

```
sudo apt update && sudo apt install -y curl apt-transport-https gnupg
curl https://couchdb.apache.org/repo/keys.asc | gpg --dearmor | sudo tee /usr/share/keyrings/couchdb-archive-keyring.gpg >/dev/null 2>&1
source /etc/os-release
echo "deb [signed-by=/usr/share/keyrings/couchdb-archive-keyring.gpg] https://apache.jfrog.io/artifactory/couchdb-deb/ ${VERSION_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/couchdb.list >/dev/null

sudo apt update
sudo apt install -y couchdb

> MagicErlangCookie: 8zplQ4cMBYma4y6
> Bind address: 0.0.0.0
> Password : password
> Password repeat : password
```