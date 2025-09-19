set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
alias hw := start-navigation
[private]
alias sim := start-simulation

[private]
default:
    @just --list --unsorted

[private]
check-husarion-webui:
    #!/bin/bash
    if ! command -v snap &> /dev/null; then
        echo "Snap is not installed. Please install Snap first and try again."
        echo "sudo apt install snapd"
        exit 1
    fi

    if ! snap list husarion-webui &> /dev/null; then
        echo "husarion-webui is not installed."
        read -p "Do you want to install husarion-webui? (y/n): " choice
        case "$choice" in
            y|Y )
                sudo snap install husarion-webui --channel=humble
                ;;
            n|N )
                echo "Installation aborted."
                exit 0
                ;;
            * )
                echo "Invalid input. Please respond with 'y' or 'n'."
                exit 1
                ;;
        esac
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit
        fi
        sudo apt-get install -y rsync sshpass inotify-tools
    fi

# start ROSbot autonomy container
start-navigation:
    #!/bin/bash
    if grep -q "Intel(R) Atom(TM) x5-Z8350" /proc/cpuinfo && [[ "${CONTROLLER}" == "mppi" ]]; then
        echo -e "\e[1;33mMPPI controller is not compatible with ROSbot 2 PRO. Please use DWB or RPP controller\e[0m"
        exit
    fi

    docker compose -f docker/compose.yaml down
    docker compose -f docker/compose.yaml pull
    docker compose -f docker/compose.yaml up

# start Gazebo simulator with autonomy
start-simulation:
    #!/bin/bash
    xhost +local:docker
    docker compose -f docker/compose.sim.yaml down
    docker compose -f docker/compose.sim.yaml pull
    docker compose -f docker/compose.sim.yaml up

start-visualization: check-husarion-webui
    #!/bin/bash
    sudo cp rosbot_navigation/layout/foxglove.json /var/snap/husarion-webui/common/foxglove-rosbot-navigation.json
    sudo snap set husarion-webui webui.layout=rosbot-navigation
    sudo husarion-webui.start

    local_ip=$(ip -o -4 addr show scope global | awk '{print $4}' | cut -d/ -f1 | head -n1)
    hostname=$(hostname)
    echo "Access the web interface at:"
    echo "  • Localhost:        http://localhost:8080/ui"
    echo "  • Local network:    http://$local_ip:8080/ui"
    echo "  • Husarnet network: http://$hostname:8080/ui"

# copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync
    #!/bin/bash
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --exclude='.docs' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' --exclude='.docs' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --exclude='.docs' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done
