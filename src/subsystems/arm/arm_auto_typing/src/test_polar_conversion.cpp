#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <stdexcept>

struct Point {
    double x;
    double y;
};

struct PolarPoint {
    double r;
    double theta;
};

// Convert Cartesian points to polar coordinates relative to origin
std::vector<PolarPoint> convert_to_polar(const std::vector<Point>& points, const Point& origin) {
    std::vector<PolarPoint> polar_points;
    
    for (const auto& point : points) {
        // Center the point relative to origin
        double centered_x = point.x - origin.x;
        double centered_y = point.y - origin.y;
        
        // Calculate r (distance)
        double r = std::sqrt(centered_x * centered_x + centered_y * centered_y);
        
        // Calculate theta (angle) - swap x and y for north-oriented (0 deg = north)
        double theta = std::atan2(centered_x, centered_y) * 180.0 / M_PI;
        
        // Convert from -180 to 180 range to 0 to 360 range
        if (theta < 0) {
            theta += 360.0;
        }
        
        polar_points.push_back({r, theta});
    }
    
    return polar_points;
}

// Map keys to their positions from CSV file
std::vector<Point> map_keys_to_positions(const std::vector<std::string>& keys, 
                                          const std::string& csv_file = "Keyboard Measurements - keyboard_measurements.csv") {
    std::map<std::string, Point> key_positions;
    
    // Read the CSV file
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open CSV file: " + csv_file);
    }
    
    std::string line;
    // Skip header row
    std::getline(file, line);
    
    // Read each row
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::stringstream ss(line);
        std::string key_name, x_str, y_str, z_str;
        
        std::getline(ss, key_name, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, z_str, ',');
        
        // Trim whitespace from strings
        x_str.erase(0, x_str.find_first_not_of(" \t\r\n"));
        x_str.erase(x_str.find_last_not_of(" \t\r\n") + 1);
        y_str.erase(0, y_str.find_first_not_of(" \t\r\n"));
        y_str.erase(y_str.find_last_not_of(" \t\r\n") + 1);
        
        if (x_str.empty() || y_str.empty()) continue;
        
        try {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            key_positions[key_name] = {x, y};
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse line for key '" << key_name 
                     << "' (x='" << x_str << "', y='" << y_str << "')" << std::endl;
            continue;
        }
    }
    
    file.close();
    
    // Map the input keys to their positions
    std::vector<Point> positions;
    for (const auto& key : keys) {
        if (key_positions.find(key) != key_positions.end()) {
            positions.push_back(key_positions[key]);
        } else {
            throw std::runtime_error("Key '" + key + "' not found in keyboard measurements");
        }
    }
    
    return positions;
}

int main() {
    try {
        // Ask user for origin key
        std::string origin_key;
        std::cout << "Enter origin key: ";
        std::getline(std::cin, origin_key);
        
        std::vector<Point> origin_points = map_keys_to_positions({origin_key});
        Point origin = origin_points[0];
        
        // Ask user for number of keys
        int num_keys;
        std::cout << "How many keys in the sequence? ";
        std::cin >> num_keys;
        std::cin.ignore(); // Clear newline from buffer
        
        // Ask user for key inputs
        std::cout << "Enter " << num_keys << " keys:" << std::endl;
        std::vector<std::string> keys;
        for (int i = 0; i < num_keys; i++) {
            std::string key;
            std::cout << "Key " << (i + 1) << ": ";
            std::getline(std::cin, key);
            keys.push_back(key);
        }
        
        // Get positions for the keys
        std::vector<Point> points = map_keys_to_positions(keys);
        
        // Convert to polar coordinates
        std::vector<PolarPoint> polar_points = convert_to_polar(points, origin);
        
        // Print results
        std::cout << "\nPolar coordinates (r, theta):" << std::endl;
        for (size_t i = 0; i < polar_points.size(); i++) {
            std::cout << keys[i] << ": r=" << polar_points[i].r 
                     << ", theta=" << polar_points[i].theta << " degrees" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
