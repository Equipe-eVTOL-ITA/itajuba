#ifndef BASE_CPP
#define BASE_CPP

struct Base{
    Eigen::Vector3d coordinates;
    std::string shape_class;  // Store the detected shape: "circulo", "triangulo", etc.
    bool is_visited = false;
};

#endif