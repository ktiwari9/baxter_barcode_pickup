// Use in class constructor
target_aquisition_service_ = node_.advertiseService("navigator", &PS_Server::service_callback, this);
initial_state_client_ = node_.serviceClient<barcode_pickup::setup_service>("barcode_pickup_setup");

// Use as public method
bool service_callback(barcode_pickup::setup_service::Request &req,
		      barcode_pickup::setup_service::Response &res)
{
    res.x = req.x
    return true;
}
