#!/bash/bin

CONTAINER_ID="main_testing_image"

PATH_TO_LOGS="/home/testing/dev_ws/logger.csv"

PATH_TO_LOCAL_LOGS="/Users/matasjones/Documents/Coding_projects/testing/testing_logs"

CMD="docker cp $CONTAINER_ID:$PATH_TO_LOGS $PATH_TO_LOCAL_LOGS"

echo "Fetching logs.."

#ssh ${USERS[1]}@${SERVER_IP} docker ps -a

source config.sh

# REMOTE_IPS[1] corresponds to the black holohover
if ping -c 1 ${SERVER_IP} &> /dev/null; then # ping -c 1 ip_address -> sends a ping to the ip address and &> /dev/null redirects any output to /dev/null effectively hiding it
    
    # Send command to devices
    ssh ${USERS[1]}@${SERVER_IP} $CMD

else
    echo "Ping to $SERVER_IP failed..."

fi # Used to indicated the end of an if statement block