#include "turtlebot3_firebase.hpp"

firebase::App* firebase_app;
const char *databasePath = "turtlebot_state";
const char *requestPath = "request";

const char *email = "minhlangstudy@gmail.com";
const char *password = "123456789";

const char *api_key = "AIzaSyD3m-cPZIxZwK59Y8q1HCLxWVOf0qVHSjQ";
const char *app_id = "1:419978511119:android:eb17bcb42e9438e41cf47a";
const char *database_url = "https://turtlebot3-3bd17-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *storage_bucket = "turtlebot3-3bd17.appspot.com";
const char *project_id = "turtlebot3-3bd17";

firebase::AppOptions options;

void InitializeFirebase()
{
    options.set_api_key(api_key);
    options.set_app_id(app_id);
    options.set_database_url(database_url);
    options.set_storage_bucket(storage_bucket);
    options.set_project_id(project_id);

    firebase_app = firebase::App::Create(options);
    if (!firebase_app)
    {
        std::cerr << "Failed to initialize Firebase app." << std::endl;
    }
}


void WaitForCompletion(const firebase::FutureBase &future, const char *name){
     while (future.status() == firebase::kFutureStatusPending)
    {
    }
    if (future.status() != firebase::kFutureStatusComplete)
    {
        printf("ERROR: %s returned an invalid result.\n", name);
    }
    else if (future.error() != 0)
    {
        printf("ERROR: %s returned error %d: %s\n", name, future.error(),
               future.error_message());
    }
}

void SetBattery(int value){
  // Ensure that the Firebase app is initialized.
    if (!firebase_app)
    {
        std::cerr << "Firebase app is not initialized." << std::endl;
        // Handle error as needed.
        return;
    }

    // Get a reference to the Firebase Realtime Database.
    firebase::database::Database *database = firebase::database::Database::GetInstance(firebase_app);
    if (!database)
    {
        std::cerr << "Failed to get the database instance." << std::endl;
        // Handle error as needed.
        return;
    }

    // Get a reference to the database location where you want to publish the value.
    firebase::database::DatabaseReference reference = database->GetReference(databasePath);
    // Set the value at the specified location.
    auto future = reference.Child("battery").SetValue(value);
    WaitForCompletion(future, "set");
}
int GetBattery(){
       // Ensure that the Firebase app is initialized.
    if (!firebase_app)
    {
        std::cerr << "Firebase app is not initialized." << std::endl;
        // Handle error as needed.
        return -1;
    }

    // Get a reference to the Firebase Realtime Database.
    firebase::database::Database *database = firebase::database::Database::GetInstance(firebase_app);
    if (!database)
    {
        std::cerr << "Failed to get the database instance." << std::endl;
        // Handle error as needed.
        return -1;
    }

    // Get a reference to the database location where you want to publish the value.
    firebase::database::DatabaseReference reference = database->GetReference(databasePath);
    // Get the value at the specified location.
    firebase::Future<firebase::database::DataSnapshot> result = reference.Child("battery").GetValue();
    WaitForCompletion(result, "get");
    const firebase::database::DataSnapshot snapshot = *result.result();
    return snapshot.value().int64_value();
} 

getRequestData getRequest(){
    // Ensure that the Firebase app is initialized.
    getRequestData myRequest;
    myRequest.id = 1;
    if (!firebase_app)
    {
        std::cerr << "Firebase app is not initialized." << std::endl;
        myRequest.id = -1;
        // Handle error as needed.
        return myRequest;
    }
       // Get a reference to the Firebase Realtime Database.
    firebase::database::Database *database = firebase::database::Database::GetInstance(firebase_app);

    if (!database)
    {
        std::cerr << "Failed to get the database instance." << std::endl;
        myRequest.id = -1;
        // Handle error as needed.
        return myRequest;
    }
     // Get the root reference location of the database.
    firebase::database::DatabaseReference dbref = database->GetReference(requestPath);
    firebase::Future<firebase::database::DataSnapshot> result = dbref.GetValue();
    WaitForCompletion(result, "get");
    const firebase::database::DataSnapshot myID = *result.result();
    //check field of id whether exist or not 
    if(!(myID.HasChild("id"))){
        std::cerr << "Don't have this field id" << std::endl;
    }
    // check value of id whether null or not
    if(!(myID.Child("id").value().is_null())){
        std::cerr << "Value is not null" << std::endl;
    }
        //check the field of x and y of the param whether exist or not 
    if(!(myID.Child("param").HasChild("x")) &&  !(myID.Child("param").HasChild("y"))){
        std::cerr << "Don't have this field x and y" << std::endl;
    }
    // check value of x whether null or not
    if(!(myID.Child("param").Child("x").value().is_null())){
        std::cerr << "Value is not null" << std::endl;
    }
    // check value of y whether null or not
    if(!(myID.Child("param").Child("y").value().is_null())){
        std::cerr << "Value is not null" << std::endl;
    }
    //debug
    // std::cerr << myID.Child("id").value().int64_value()<< std::endl;
    // std::cerr << myID.Child("param").Child("x").value().int64_value()<< std::endl;
    // std::cerr << myID.Child("param").Child("y").value().int64_value()<< std::endl;
    //Assign a value to the variable myRequest
    myRequest.id = myID.Child("id").value().int64_value();
    firebase::Variant x = myID.Child("param").Child("x").value();
    firebase::Variant y = myID.Child("param").Child("y").value();
    myRequest.xPosition = (x.is_int64()) ? x.int64_value() : x.double_value();
    myRequest.yPosition = (y.is_int64()) ? y.int64_value() : y.double_value();
    return myRequest; 
}