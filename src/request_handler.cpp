#include "request_handler.hpp"
#include "turtlebot3_firebase.hpp"
#include "chrono"
std::chrono::steady_clock::time_point startTime;
std::chrono::steady_clock::time_point endTime;
RequestHandler::RequestHandler() : Node("user_input_publisher"), ValueListener()
{
    InitializeFirebase();
    database = firebase::database::Database::GetInstance(firebase_app);
    dbref = database->GetReference(requestPath);
    dbref.AddValueListener(this);
    timer = this->create_wall_timer(50ms, std::bind(&RequestHandler::handlerCallback, this));
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
    request.xPosition = (x.is_int64()) ? x.int64_value() : x.double_value();
    request.yPosition = (y.is_int64()) ? y.int64_value() : y.double_value();
    RCLCPP_INFO(get_logger(), "Got request with ID %d and position (%f;%f)", request.id, request.xPosition, request.yPosition);
}

void RequestHandler::OnCancelled(const firebase::database::Error &error_code,
                                 const char *error_message)
{
}

bool caculateTimeout(){
    endTime = std::chrono::steady_clock::now();
    auto elaspedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime)*1000;
    RCLCPP_INFO(get_logger(), "Cancel! Elapsed time: %d", elaspedTime.count());
    if(elaspedTime > std::chrono::seconds(5))
    {
        return true;
    }
    return false;
}

void RequestHandler::handlerCallback()
{
    static auto goal_pose = geometry_msgs::msg::PoseStamped();

    switch (state)
    {
    case 0:
        if (request.id == 1)
        {
            needPublishing = true;
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = now();
            goal_pose.pose.position.x = request.xPosition;
            goal_pose.pose.position.y = request.yPosition;
            goal_pose.pose.position.z = 0.0;
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = 1;
            goal_pose.pose.orientation.w = 0;
            this->nav.startNavigation(goal_pose);
            state = 1;
            RCLCPP_INFO(get_logger(), "Start navigating to pick up station!\n");
        }
        setStatus(true);
        break;
    case 1:
        if (this->nav.doneNavigate())
        {
            startTime = std::chrono::steady_clock::now();
            state = 2;
            setStatus(false);
            RCLCPP_INFO(get_logger(), "Reached pick up station!\n");
        }

        if (request.id == 0)    
        {
            this->nav.cancelNavigation();
            state = 0;
            setStatus(false);
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        break;
    case 2:
        if(caculateTimeout){
            state = 0;
            setStatus(true);
            RCLCPP_INFO(get_logger(), "TimeOut!\n");
        }
        if (request.id == 2)
        {
            needPublishing = true;
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = now();
            goal_pose.pose.position.x = request.xPosition;
            goal_pose.pose.position.y = request.yPosition;
            goal_pose.pose.position.z = 0;
            goal_pose.pose.orientation.x = 0;
            goal_pose.pose.orientation.y = 0;
            goal_pose.pose.orientation.z = 0;
            goal_pose.pose.orientation.w = 0;
            this->nav.startNavigation(goal_pose);
            state = 3;
            setStatus(false);
            RCLCPP_INFO(get_logger(), "Start navigating to destination!\n");
        }
        break;
    case 3:
        if (this->nav.doneNavigate())
        {
            state = 0;
            RCLCPP_INFO(get_logger(), "Reached destination!\n");
        }

        if (request.id == 0)
        {
            this->nav.cancelNavigation();
            state = 0;
            RCLCPP_INFO(get_logger(), "Cancel!\n");
        }
        setStatus(false);
        break;
    default:
        break;
    }
}