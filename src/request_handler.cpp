#include "request_handler.hpp"

RequestHandler::RequestHandler() : Node("user_input_publisher"), ValueListener()
{
    InitializeFirebase();
    database = firebase::database::Database::GetInstance(firebase_app);
    dbref = database->GetReference(requestPath);
    dbref.AddValueListener(this);
    timer = this->create_wall_timer(50ms, std::bind(&RequestHandler::handlerCallback, this));
    setStatus(true);
    speak("Waiting for request");
}

RequestHandler::~RequestHandler()
{
    delete database;
}

void RequestHandler::OnValueChanged(
    const firebase::database::DataSnapshot &snapshot)
{
    request.id = snapshot.Child("id").value().int64_value();
    firebase::Variant x = snapshot.Child("param").Child("x").value();
    firebase::Variant y = snapshot.Child("param").Child("y").value();
    firebase::Variant yaw = snapshot.Child("param").Child("yaw").value();
    request.xPosition = (x.is_int64()) ? x.int64_value() : x.double_value();
    request.yPosition = (y.is_int64()) ? y.int64_value() : y.double_value();
    request.yaw = (yaw.is_int64()) ? yaw.int64_value() : yaw.double_value();
    request.station.destinantionStation = snapshot.Child("station").Child("desc").value().string_value();
    request.station.id = snapshot.Child("station").Child("id").value().int64_value();
    request.station.nameStation = snapshot.Child("station").Child("name").value().string_value();

    RCLCPP_INFO(get_logger(), "Got request with ID %d and position (%f;%f) - %f at %s", request.id, request.xPosition, request.yPosition, request.yaw, request.station.nameStation.c_str());
}

void RequestHandler::OnCancelled(const firebase::database::Error &error_code,
                                 const char *error_message)
{
}

void RequestHandler::handlerCallback()
{
    static auto goal_pose = geometry_msgs::msg::PoseStamped();
    static char text[1024];

    switch (state)
    {
    case 0:
        if (request.id == 1)
        {
            sprintf(text, "Start navigating to %s station", request.station.nameStation.c_str());
            speak((const char*) text);
            needPublishing = true;
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = now();
            goal_pose.pose.position.x = request.xPosition;
            goal_pose.pose.position.y = request.yPosition;
            goal_pose.pose.position.z = 0.0;
            double radians = request.yaw * M_PI / 180.0;
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, radians);
            goal_pose.pose.orientation = tf2::toMsg(quaternion);
            this->nav.startNavigation(goal_pose);
            state = 1;
            setStatus(false);
            RCLCPP_INFO(get_logger(), "Start navigating to pick up station!\n");
        }
        break;
    case 1:
        if (this->nav.doneNavigate())
        {
            state = 2;
            sprintf(text, "Reached pick %s station, please choose your destination", request.station.nameStation.c_str());
            speak((const char*) text);
            RCLCPP_INFO(get_logger(), "Reached pick up station!\n");
        }

        if (request.id == 0)
        {
            this->nav.cancelNavigation();
            state = 0;
            setStatus(true);
            speak("Cancelled");
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    case 2:
        if (request.id == 2)
        {
            sprintf(text, "Start navigating to %s desination", request.station.nameStation.c_str());
            speak((const char*) text);
            needPublishing = true;
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = now();
            goal_pose.pose.position.x = request.xPosition;
            goal_pose.pose.position.y = request.yPosition;
            goal_pose.pose.position.z = 0.0;
            double radians = request.yaw * M_PI / 180.0;
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, radians);
            goal_pose.pose.orientation = tf2::toMsg(quaternion);
            this->nav.startNavigation(goal_pose);
            state = 3;
            RCLCPP_INFO(get_logger(), "Start navigating to destination!\n");
        }
        if (request.id == 0)
        {
            state = 0;
            setStatus(true);
            speak("Cancelled");
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    case 3:
        if (this->nav.doneNavigate())
        {
            state = 0;
            setStatus(true);
            sprintf(text, "Reached %s desination", request.station.nameStation.c_str());
            speak((const char*) text);
            RCLCPP_INFO(get_logger(), "Reached destination!\n");
        }

        if (request.id == 0)
        {
            this->nav.cancelNavigation();
            state = 0;
            setStatus(true);
            speak("Cancelled");
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    default:
        break;
    }
}
