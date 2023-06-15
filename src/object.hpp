#include <rclcpp/rclcpp.hpp>

#include <random>

// Object with uniform orbit
class Object
{
public:
	Object(int id)
	{
		id_ = id;

		// Initialize random engine
		std::random_device rd;
		std::mt19937_64 gen(rd());
		std::normal_distribution<double> dist(0, 1);

		// Initialize object states
		footprint_radius_ = 2.*dist(gen) + 1.;
		footprint_shape_ = M_PI/32.*dist(gen) + M_PI/8.;
		
		orbital_center_x_ = 10.*dist(gen) - 5.;
		orbital_center_y_ = 10.*dist(gen) - 5.;
		orbital_radius_ = 20.*dist(gen) - 10.;
		orbital_position_ = 6.28*dist(gen);
		orbital_velocity_ = 0.1*dist(gen) + 0.2;
	}

	void update(float dt)
	{
		orbital_position_ += orbital_velocity_*dt;
		orbital_position_ = fmod(orbital_position_, 2*M_PI);
	}

	int get_id() { return id_; }

	std::tuple<float, float, float, std::vector< std::tuple<float, float> > > get_states()
	{
		// Pose
		float x = orbital_center_x_ + orbital_radius_*cos(orbital_position_);
		float y = orbital_center_y_ + orbital_radius_*sin(orbital_position_);
		float Y = fmod(orbital_position_ + (orbital_velocity_ > 0 ? 1. : -1.)*M_PI_2, 2*M_PI);

		// Footprint
		std::vector< std::tuple<float, float> > footprint;
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y + footprint_shape_),
			y + footprint_radius_*sin(Y + footprint_shape_)
		));
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y - footprint_shape_),
			y + footprint_radius_*sin(Y - footprint_shape_)
		));
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y + footprint_shape_ + M_PI),
			y + footprint_radius_*sin(Y + footprint_shape_ + M_PI)
		));
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y - footprint_shape_ - M_PI),
			y + footprint_radius_*sin(Y - footprint_shape_ - M_PI)
		));

		return std::make_tuple(x, y, Y, footprint);
	}

private:
	int id_;

	float footprint_radius_;
	float footprint_shape_;

	float orbital_center_x_;
    float orbital_center_y_;
    float orbital_radius_;
    float orbital_position_;
    float orbital_velocity_;
};