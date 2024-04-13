#include "request_handler.hpp"

RequestHandler::RequestHandler() : Node("user_input_publisher"), ValueListener()
{
    InitializeFirebase();
    database = firebase::database::Database::GetInstance(firebase_app);
    dbref = database->GetReference(requestPath);
    dbref.AddValueListener(this);
    timer = this->create_wall_timer(50ms, std::bind(&RequestHandler::handlerCallback, this));
    setStatus(true);
    isReachStation(0);
    needPublishing = true;
    // speak("Waiting for request");
}

RequestHandler::~RequestHandler()
{
    delete database;
}

void RequestHandler::OnValueChanged(
    const firebase::database::DataSnapshot &snapshot)
{
    request.id = snapshot.Child("id").value().int64_value();
    if (request.id == 3)
    {
        multiple_request.clear();
        parseMultiStation(snapshot.Child("multipleStation").value().string_value(), multiple_request);
        RCLCPP_INFO(get_logger(), "Got request with ID %d", request.id);
        for (int i = 0; i < multiple_request.size(); i++)
        {
            RCLCPP_INFO(get_logger(), "%s - %f ; %f ; %f\n", multiple_request[i].station.nameStation.c_str(), multiple_request[i].xPosition, multiple_request[i].yPosition, multiple_request[i].yaw);
        }
    }
    if (request.id == 0 || request.id == 1 || request.id == 2)
    {
        firebase::Variant x = snapshot.Child("param").Child("x").value();
        firebase::Variant y = snapshot.Child("param").Child("y").value();
        firebase::Variant yaw = snapshot.Child("param").Child("yaw").value();
        request.xPosition = (x.is_int64()) ? x.int64_value() : x.double_value();
        request.yPosition = (y.is_int64()) ? y.int64_value() : y.double_value();
        request.yaw = (yaw.is_int64()) ? yaw.int64_value() : yaw.double_value();
        request.station.description = snapshot.Child("station").Child("description").value().string_value();
        request.station.id = snapshot.Child("station").Child("id").value().int64_value();
        request.station.nameStation = snapshot.Child("station").Child("name").value().string_value();

        RCLCPP_INFO(get_logger(), "Got request with ID %d and position (%f;%f) - %f at %s", request.id, request.xPosition, request.yPosition, request.yaw, request.station.nameStation.c_str());
    }
}

void RequestHandler::OnCancelled(const firebase::database::Error &error_code,
                                 const char *error_message)
{
}

void RequestHandler::postHttpReachGoal()
{
    CURL *curl;
    CURLcode res;

    /* In windows, this will init the winsock stuff */
    curl_global_init(CURL_GLOBAL_ALL);

    /* get a curl handle */
    curl = curl_easy_init();
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_URL, "https://ultimatetech.live/v1/tours/mockup");
        /* Now specify the POST data */
        const char *json = "{\"fromStation\": \"1\", \"toStation\": \"10\"}";
        struct curl_slist *slist1 = NULL;
        slist1 = curl_slist_append(slist1, "Content-Type: application/json");
        slist1 = curl_slist_append(slist1, "Accept: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, slist1);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);

        /* Perform the request, res will get the return code */
        res = curl_easy_perform(curl);
        /* Check for errors */
        if (res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                    curl_easy_strerror(res));

        /* always cleanup */
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
}

geometry_msgs::msg::PoseStamped RequestHandler::convert2GeometryMsg(double x, double y, double yaw)
{
    auto goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = now();
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal_pose.pose.position.z = 0.0;
    double radians = yaw * M_PI / 180.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, radians);
    goal_pose.pose.orientation = tf2::toMsg(quaternion);
    return goal_pose;
}

void RequestHandler::handlerMultipleStation()
{
    static char text[1024];

    std::vector<std::shared_ptr<geometry_msgs::msg::PoseStamped>> allposes;
    for (int i = 0; i < multiple_request.size(); i++)
    {
        allposes.push_back(std::make_shared<geometry_msgs::msg::PoseStamped>(convert2GeometryMsg(multiple_request[i].yPosition, multiple_request[i].xPosition, multiple_request[i].yaw)));
    }
    std::vector<int> optimized_path = path_cli.getOptimizedPath(allposes);
    for (int vertex : optimized_path)
    {
        sprintf(text, "Start navigating to %s station", multiple_request[vertex].station.nameStation.c_str());
        // speak((const char*) text);
        RCLCPP_INFO(get_logger(), "Navigating to multiple stations!\n");
        this->nav.startNavigation(*allposes[vertex]);
        while (!nav.doneNavigate()) {
            if (request.id == 0) {
                this->nav.cancelNavigation();
                setStatus(true);
                // speak("Cancelled");
                isReachStation(0);
                RCLCPP_INFO(get_logger(), "Cancel!\n");
                return;
            }
        }
    }
    multiple_request.clear();
}

void RequestHandler::handlerCallback()
{
    static char text[1024];

    switch (state)
    {
    case 0:
        if (request.id == 1)
        {
            sprintf(text, "Start navigating to %s station", request.station.nameStation.c_str());
            // speak((const char*) text);
            this->nav.startNavigation(convert2GeometryMsg(request.xPosition, request.yPosition, request.yaw));
            state = 1;
            setStatus(false);
            isReachStation(0);
            RCLCPP_INFO(get_logger(), "Start navigating to pick up station!\n");
        }

        if (request.id == 3)
        {
            setStatus(false);
            handlerMultipleStation();
        }
        break;
    case 1:
        if (this->nav.doneNavigate())
        {
            state = 2;
            sprintf(text, "Reached pick %s station, please choose your destination", request.station.nameStation.c_str());
            isReachStation(1);
            // speak((const char*) text);
            RCLCPP_INFO(get_logger(), "Reached pick up station!\n");
        }

        if (request.id == 0)
        {
            this->nav.cancelNavigation();
            state = 0;
            setStatus(true);
            // speak("Cancelled");
            isReachStation(0);
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    case 2:
        if (request.id == 2)
        {
            sprintf(text, "Start navigating to %s desination", request.station.nameStation.c_str());
            // speak((const char*) text);
            this->nav.startNavigation(convert2GeometryMsg(request.xPosition, request.yPosition, request.yaw));
            state = 3;
            isReachStation(1);
            RCLCPP_INFO(get_logger(), "Start navigating to destination!\n");
        }
        if (request.id == 0)
        {
            state = 0;
            setStatus(true);
            // speak("Cancelled");
            isReachStation(0);
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    case 3:
        if (this->nav.doneNavigate())
        {
            state = 0;
            setStatus(true);
            sprintf(text, "Reached %s desination", request.station.nameStation.c_str());
            // speak((const char*) text);
            isReachStation(2);
            RCLCPP_INFO(get_logger(), "Reached destination!\n");
        }

        if (request.id == 0)
        {
            this->nav.cancelNavigation();
            state = 0;
            setStatus(true);
            // speak("Cancelled");
            isReachStation(0);
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    default:
        break;
    }
}
