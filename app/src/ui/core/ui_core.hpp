#pragma once

#include "coord_transform.hpp"

class Document;
struct NetworkEventBus;
class Project;

struct UiCore {
    const Document& doc;
    NetworkEventBus& bus;
    Project& project;
    CoordTransform xf;
};