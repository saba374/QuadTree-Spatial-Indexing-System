#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <algorithm>

using namespace std;

// -----------------------------------------------------------
// LOCATION STRUCTURE (Real-world representation)
// -----------------------------------------------------------
struct Location {
    double latitude;   // X coordinate
    double longitude;  // Y coordinate
    int id;
    string name;
    string type;       // "CAB", "RESTAURANT", "USER", "HOSPITAL", etc.
    
    Location(double lat = 0, double lon = 0, int id = 0, 
             string name = "", string type = "POINT") 
        : latitude(lat), longitude(lon), id(id), name(name), type(type) {}
    
    void print() const {
        cout << "[" << type << "] " << name 
             << " (ID: " << id << ") - Lat: " << latitude 
             << ", Lon: " << longitude;
    }
    
    double distanceTo(const Location& other) const {
        return sqrt(pow(latitude - other.latitude, 2) + 
                   pow(longitude - other.longitude, 2));
    }
};

// -----------------------------------------------------------
// BOUNDING BOX STRUCTURE
// -----------------------------------------------------------
struct BoundingBox {
    double centerLat, centerLon;  // Center coordinates
    double halfWidth, halfHeight;  // Half dimensions
    
    BoundingBox(double lat = 0, double lon = 0, double w = 0, double h = 0) 
        : centerLat(lat), centerLon(lon), halfWidth(w), halfHeight(h) {}
    
    bool contains(const Location& loc) const {
        return (loc.latitude >= centerLat - halfWidth && 
                loc.latitude <= centerLat + halfWidth &&
                loc.longitude >= centerLon - halfHeight && 
                loc.longitude <= centerLon + halfHeight);
    }
    
    bool intersects(const BoundingBox& range) const {
        return !(range.centerLat - range.halfWidth > centerLat + halfWidth ||
                 range.centerLat + range.halfWidth < centerLat - halfWidth ||
                 range.centerLon - range.halfHeight > centerLon + halfHeight ||
                 range.centerLon + range.halfHeight < centerLon - halfHeight);
    }
    
    void print() const {
        cout << "BBox[(" << centerLat - halfWidth << "," 
             << centerLon - halfHeight << ") to (" 
             << centerLat + halfWidth << "," 
             << centerLon + halfHeight << ")]";
    }
};

// -----------------------------------------------------------
// QUADTREE CLASS FOR SPATIAL INDEXING
// -----------------------------------------------------------
class QuadTree {
private:
    BoundingBox boundary;
    int capacity;
    vector<Location> locations;
    bool divided;
    int depth;
    
    // Four child quadrants
    QuadTree* northeast;
    QuadTree* northwest;
    QuadTree* southeast;
    QuadTree* southwest;
    
    void subdivide() {
        double lat = boundary.centerLat;
        double lon = boundary.centerLon;
        double w = boundary.halfWidth / 2;
        double h = boundary.halfHeight / 2;
        
        northeast = new QuadTree(BoundingBox(lat + w, lon - h, w, h), capacity, depth + 1);
        northwest = new QuadTree(BoundingBox(lat - w, lon - h, w, h), capacity, depth + 1);
        southeast = new QuadTree(BoundingBox(lat + w, lon + h, w, h), capacity, depth + 1);
        southwest = new QuadTree(BoundingBox(lat - w, lon + h, w, h), capacity, depth + 1);
        
        divided = true;
    }

public:
    QuadTree(BoundingBox boundary, int capacity = 4, int depth = 0) 
        : boundary(boundary), capacity(capacity), divided(false), depth(depth),
          northeast(NULL), northwest(NULL), southeast(NULL), southwest(NULL) {}
    
    ~QuadTree() {
        if (divided) {
            delete northeast;
            delete northwest;
            delete southeast;
            delete southwest;
        }
    }
    
    // OPERATION 1: INSERT A LOCATION
    bool insert(const Location& loc) {
        if (!boundary.contains(loc)) {
            return false;
        }
        
        if (locations.size() < (size_t)capacity && !divided) {
            locations.push_back(loc);
            return true;
        }
        
        if (!divided) {
            subdivide();
            for (size_t i = 0; i < locations.size(); i++) {
                northeast->insert(locations[i]) || northwest->insert(locations[i]) ||
                southeast->insert(locations[i]) || southwest->insert(locations[i]);
            }
            locations.clear();
        }
        
        return northeast->insert(loc) || northwest->insert(loc) ||
               southeast->insert(loc) || southwest->insert(loc);
    }
    
    // OPERATION 2: SEARCH FOR COORDINATES OF A SPECIFIC LOCATION
    Location* searchByName(const string& name) {
        for (size_t i = 0; i < locations.size(); i++) {
            if (locations[i].name == name) {
                return &locations[i];
            }
        }
        
        if (divided) {
            Location* result;
            result = northeast->searchByName(name);
            if (result) return result;
            
            result = northwest->searchByName(name);
            if (result) return result;
            
            result = southeast->searchByName(name);
            if (result) return result;
            
            result = southwest->searchByName(name);
            if (result) return result;
        }
        
        return NULL;
    }
    
    Location* searchById(int id) {
        for (size_t i = 0; i < locations.size(); i++) {
            if (locations[i].id == id) {
                return &locations[i];
            }
        }
        
        if (divided) {
            Location* result;
            result = northeast->searchById(id);
            if (result) return result;
            
            result = northwest->searchById(id);
            if (result) return result;
            
            result = southeast->searchById(id);
            if (result) return result;
            
            result = southwest->searchById(id);
            if (result) return result;
        }
        
        return NULL;
    }
    
    // OPERATION 3: COUNT LOCATIONS IN A CONTIGUOUS AREA
    int countInRange(const BoundingBox& range) const {
        if (!boundary.intersects(range)) {
            return 0;
        }
        
        int count = 0;
        for (size_t i = 0; i < locations.size(); i++) {
            if (range.contains(locations[i])) {
                count++;
            }
        }
        
        if (divided) {
            count += northeast->countInRange(range);
            count += northwest->countInRange(range);
            count += southeast->countInRange(range);
            count += southwest->countInRange(range);
        }
        
        return count;
    }
    
    // ADDITIONAL: Query all locations within a range
    void queryRange(const BoundingBox& range, vector<Location>& found) const {
        if (!boundary.intersects(range)) {
            return;
        }
        
        for (size_t i = 0; i < locations.size(); i++) {
            if (range.contains(locations[i])) {
                found.push_back(locations[i]);
            }
        }
        
        if (divided) {
            northeast->queryRange(range, found);
            northwest->queryRange(range, found);
            southeast->queryRange(range, found);
            southwest->queryRange(range, found);
        }
    }
    
    // Find nearest location (useful for ride-sharing apps)
    Location* findNearest(const Location& target, double& minDist) {
        Location* nearest = NULL;
        
        for (size_t i = 0; i < locations.size(); i++) {
            double dist = locations[i].distanceTo(target);
            if (dist < minDist && dist > 0) {
                minDist = dist;
                nearest = &locations[i];
            }
        }
        
        if (divided) {
            Location* temp;
            temp = northeast->findNearest(target, minDist);
            if (temp) nearest = temp;
            
            temp = northwest->findNearest(target, minDist);
            if (temp) nearest = temp;
            
            temp = southeast->findNearest(target, minDist);
            if (temp) nearest = temp;
            
            temp = southwest->findNearest(target, minDist);
            if (temp) nearest = temp;
        }
        
        return nearest;
    }
    
    // Find K nearest neighbors
    void findKNearest(const Location& target, int k, vector<pair<double, Location*> >& results) {
        for (size_t i = 0; i < locations.size(); i++) {
            double dist = locations[i].distanceTo(target);
            if (dist > 0) {
                results.push_back(make_pair(dist, &locations[i]));
            }
        }
        
        if (divided) {
            northeast->findKNearest(target, k, results);
            northwest->findKNearest(target, k, results);
            southeast->findKNearest(target, k, results);
            southwest->findKNearest(target, k, results);
        }
    }
    
    // Get all locations of a specific type
    void getLocationsByType(const string& type, vector<Location>& result) const {
        for (size_t i = 0; i < locations.size(); i++) {
            if (locations[i].type == type) {
                result.push_back(locations[i]);
            }
        }
        
        if (divided) {
            northeast->getLocationsByType(type, result);
            northwest->getLocationsByType(type, result);
            southeast->getLocationsByType(type, result);
            southwest->getLocationsByType(type, result);
        }
    }
    
    int countLocations() const {
        int count = locations.size();
        if (divided) {
            count += northeast->countLocations();
            count += northwest->countLocations();
            count += southeast->countLocations();
            count += southwest->countLocations();
        }
        return count;
    }
    
    void display(int d = 0) const {
        for (int i = 0; i < d * 2; i++) cout << " ";
        cout << "Node[Depth " << depth << "] - " << locations.size() << " locations" << endl;
        
        for (size_t i = 0; i < locations.size(); i++) {
            for (int j = 0; j < d * 2; j++) cout << " ";
            cout << "  ";
            locations[i].print();
            cout << endl;
        }
        
        if (divided) {
            for (int i = 0; i < d * 2; i++) cout << " ";
            cout << "|- NE" << endl;
            northeast->display(d + 1);
            
            for (int i = 0; i < d * 2; i++) cout << " ";
            cout << "|- NW" << endl;
            northwest->display(d + 1);
            
            for (int i = 0; i < d * 2; i++) cout << " ";
            cout << "|- SE" << endl;
            southeast->display(d + 1);
            
            for (int i = 0; i < d * 2; i++) cout << " ";
            cout << "|- SW" << endl;
            southwest->display(d + 1);
        }
    }
};

// -----------------------------------------------------------
// UTILITY FUNCTIONS
// -----------------------------------------------------------
void printHeader(const string& title) {
    cout << "\n+=========================================================+" << endl;
    cout << "| " << left << setw(55) << title << " |" << endl;
    cout << "+=========================================================+" << endl;
}

void printDivider() {
    cout << "-----------------------------------------------------------" << endl;
}

void clearScreen() {
    system("cls");
}

void pause() {
    cout << "\nPress Enter to continue...";
    cin.ignore();
    cin.get();
}

bool compareDistance(const pair<double, Location*>& a, const pair<double, Location*>& b) {
    return a.first < b.first;
}

// -----------------------------------------------------------
// APPLICATION SCENARIOS
// -----------------------------------------------------------
void loadSampleData(QuadTree* qt, int& nextId) {
    // Ride-sharing app: Cabs
    qt->insert(Location(25.5, 68.3, nextId++, "Cab-Alpha", "CAB"));
    qt->insert(Location(45.2, 88.7, nextId++, "Cab-Beta", "CAB"));
    qt->insert(Location(75.8, 45.2, nextId++, "Cab-Gamma", "CAB"));
    qt->insert(Location(120.3, 130.5, nextId++, "Cab-Delta", "CAB"));
    qt->insert(Location(160.7, 170.2, nextId++, "Cab-Epsilon", "CAB"));
    
    // Restaurants
    qt->insert(Location(50.0, 50.0, nextId++, "Pizza Palace", "RESTAURANT"));
    qt->insert(Location(150.0, 150.0, nextId++, "Burger King", "RESTAURANT"));
    qt->insert(Location(100.0, 100.0, nextId++, "Sushi Bar", "RESTAURANT"));
    qt->insert(Location(80.0, 120.0, nextId++, "Taco Bell", "RESTAURANT"));
    
    // Hospitals (Disaster Management)
    qt->insert(Location(30.0, 180.0, nextId++, "City Hospital", "HOSPITAL"));
    qt->insert(Location(170.0, 30.0, nextId++, "Central Medical", "HOSPITAL"));
    qt->insert(Location(100.0, 180.0, nextId++, "Emergency Care", "HOSPITAL"));
    
    // User locations (Facebook Nearby Friends)
    qt->insert(Location(95.0, 95.0, nextId++, "User-John", "USER"));
    qt->insert(Location(105.0, 105.0, nextId++, "User-Sarah", "USER"));
    qt->insert(Location(50.0, 150.0, nextId++, "User-Mike", "USER"));
    qt->insert(Location(150.0, 50.0, nextId++, "User-Emma", "USER"));
}

// -----------------------------------------------------------
// MENU SYSTEM
// -----------------------------------------------------------
void displayMenu() {
    cout << "\n+=========================================================+" << endl;
    cout << "|           SPATIAL INDEXING - OPERATION MENU             |" << endl;
    cout << "+=========================================================+" << endl;
    cout << "| CORE OPERATIONS:                                        |" << endl;
    cout << "| 1. Insert Location                                      |" << endl;
    cout << "| 2. Search Location (by Name)                            |" << endl;
    cout << "| 3. Count Locations in Area                              |" << endl;
    cout << "|                                                         |" << endl;
    cout << "| REAL-WORLD APPLICATIONS:                                |" << endl;
    cout << "| 4. Find Nearest Cab (Ride-sharing)                      |" << endl;
    cout << "| 5. Find K Nearest Restaurants                           |" << endl;
    cout << "| 6. Nearby Friends (Social Media)                        |" << endl;
    cout << "| 7. Emergency: Find Nearest Hospital                     |" << endl;
    cout << "|                                                         |" << endl;
    cout << "| UTILITIES:                                              |" << endl;
    cout << "| 8. Display Tree Structure                               |" << endl;
    cout << "| 9. Show All Locations                                   |" << endl;
    cout << "| 10. Load Sample Data                                    |" << endl;
    cout << "| 11. Statistics                                          |" << endl;
    cout << "| 0. Exit                                                 |" << endl;
    cout << "+=========================================================+" << endl;
    cout << "Enter choice: ";
}

// -----------------------------------------------------------
// MAIN FUNCTION
// -----------------------------------------------------------
int main() {
    srand(time(NULL));
    
    BoundingBox boundary(100, 100, 100, 100);
    QuadTree* qt = new QuadTree(boundary, 4);
    
    int nextId = 1;
    int choice;
    bool running = true;
    
    clearScreen();
    printHeader("QUADTREE SPATIAL INDEXING SYSTEM");
    cout << "\nInitialized QuadTree for spatial indexing" << endl;
    cout << "Coverage Area: 200 x 200 units (Lat: 0-200, Lon: 0-200)" << endl;
    cout << "Max locations per node: 4" << endl;
    pause();
    
    while (running) {
        clearScreen();
        printHeader("SPATIAL INDEXING WITH QUADTREE");
        cout << "Total Locations: " << qt->countLocations() << endl;
        displayMenu();
        cin >> choice;
        cin.ignore();
        
        switch (choice) {
            case 1: {  // Insert Location
                double lat, lon;
                string name, type;
                
                cout << "\n--- INSERT LOCATION ---" << endl;
                cout << "Name: ";
                getline(cin, name);
                cout << "Type (CAB/RESTAURANT/USER/HOSPITAL): ";
                getline(cin, type);
                cout << "Latitude (0-200): ";
                cin >> lat;
                cout << "Longitude (0-200): ";
                cin >> lon;
                
                Location loc(lat, lon, nextId, name, type);
                if (qt->insert(loc)) {
                    cout << "\n[SUCCESS] Location inserted!" << endl;
                    loc.print();
                    nextId++;
                } else {
                    cout << "\n[ERROR] Location outside boundary!" << endl;
                }
                pause();
                break;
            }
            
            case 2: {  // Search Location
                string name;
                cout << "\n--- SEARCH LOCATION ---" << endl;
                cout << "Enter location name: ";
                getline(cin, name);
                
                Location* result = qt->searchByName(name);
                if (result) {
                    cout << "\n[FOUND] ";
                    result->print();
                    cout << endl;
                } else {
                    cout << "\n[NOT FOUND] No location with that name." << endl;
                }
                pause();
                break;
            }
            
            case 3: {  // Count in Area
                double lat, lon, w, h;
                cout << "\n--- COUNT LOCATIONS IN AREA ---" << endl;
                cout << "Center Latitude: ";
                cin >> lat;
                cout << "Center Longitude: ";
                cin >> lon;
                cout << "Half-Width: ";
                cin >> w;
                cout << "Half-Height: ";
                cin >> h;
                
                BoundingBox area(lat, lon, w, h);
                int count = qt->countInRange(area);
                
                cout << "\n[RESULT] " << count << " location(s) found in area" << endl;
                
                vector<Location> found;
                qt->queryRange(area, found);
                for (size_t i = 0; i < found.size(); i++) {
                    cout << "  ";
                    found[i].print();
                    cout << endl;
                }
                pause();
                break;
            }
            
            case 4: {  // Find Nearest Cab
                double lat, lon;
                cout << "\n--- FIND NEAREST CAB (Ride-sharing) ---" << endl;
                cout << "Your Latitude: ";
                cin >> lat;
                cout << "Your Longitude: ";
                cin >> lon;
                
                Location user(lat, lon, -1, "You", "USER");
                double minDist = 1e9;
                
                vector<Location> cabs;
                qt->getLocationsByType("CAB", cabs);
                
                if (cabs.empty()) {
                    cout << "\n[INFO] No cabs available in the area." << endl;
                } else {
                    Location* nearest = NULL;
                    double minD = 1e9;
                    
                    for (size_t i = 0; i < cabs.size(); i++) {
                        double d = cabs[i].distanceTo(user);
                        if (d < minD) {
                            minD = d;
                            nearest = &cabs[i];
                        }
                    }
                    
                    if (nearest) {
                        cout << "\n[FOUND] Nearest cab:" << endl;
                        nearest->print();
                        cout << "\nDistance: " << fixed << setprecision(2) 
                             << minD << " units" << endl;
                        cout << "Estimated arrival: " << (minD / 10) 
                             << " minutes" << endl;
                    }
                }
                pause();
                break;
            }
            
            case 5: {  // K Nearest Restaurants
                double lat, lon;
                int k;
                cout << "\n--- FIND NEAREST RESTAURANTS ---" << endl;
                cout << "Your Latitude: ";
                cin >> lat;
                cout << "Your Longitude: ";
                cin >> lon;
                cout << "How many restaurants? ";
                cin >> k;
                
                Location user(lat, lon, -1, "You", "USER");
                vector<pair<double, Location*> > results;
                qt->findKNearest(user, k, results);
                
                // Filter only restaurants
                vector<pair<double, Location*> > restaurants;
                for (size_t i = 0; i < results.size(); i++) {
                    if (results[i].second->type == "RESTAURANT") {
                        restaurants.push_back(results[i]);
                    }
                }
                
                sort(restaurants.begin(), restaurants.end(), compareDistance);
                
                cout << "\n[RESULTS] Top " << min(k, (int)restaurants.size()) 
                     << " nearest restaurants:" << endl;
                     
                for (size_t i = 0; i < (size_t)k && i < restaurants.size(); i++) {
                    cout << "\n" << (i+1) << ". ";
                    restaurants[i].second->print();
                    cout << "\n   Distance: " << fixed << setprecision(2) 
                         << restaurants[i].first << " units" << endl;
                }
                pause();
                break;
            }
            
            case 6: {  // Nearby Friends
                double lat, lon, radius;
                cout << "\n--- NEARBY FRIENDS (Social Media) ---" << endl;
                cout << "Your Latitude: ";
                cin >> lat;
                cout << "Your Longitude: ";
                cin >> lon;
                cout << "Search radius: ";
                cin >> radius;
                
                BoundingBox area(lat, lon, radius, radius);
                vector<Location> nearby;
                qt->queryRange(area, nearby);
                
                cout << "\n[RESULTS] Friends nearby:" << endl;
                int count = 0;
                for (size_t i = 0; i < nearby.size(); i++) {
                    if (nearby[i].type == "USER") {
                        cout << "  ";
                        nearby[i].print();
                        cout << endl;
                        count++;
                    }
                }
                
                if (count == 0) {
                    cout << "  No friends nearby." << endl;
                }
                pause();
                break;
            }
            
            case 7: {  // Emergency: Nearest Hospital
                double lat, lon;
                cout << "\n--- EMERGENCY: FIND NEAREST HOSPITAL ---" << endl;
                cout << "Emergency Latitude: ";
                cin >> lat;
                cout << "Emergency Longitude: ";
                cin >> lon;
                
                Location emergency(lat, lon, -1, "Emergency", "EMERGENCY");
                vector<Location> hospitals;
                qt->getLocationsByType("HOSPITAL", hospitals);
                
                if (hospitals.empty()) {
                    cout << "\n[ALERT] No hospitals in database!" << endl;
                } else {
                    Location* nearest = NULL;
                    double minD = 1e9;
                    
                    for (size_t i = 0; i < hospitals.size(); i++) {
                        double d = hospitals[i].distanceTo(emergency);
                        if (d < minD) {
                            minD = d;
                            nearest = &hospitals[i];
                        }
                    }
                    
                    cout << "\n[URGENT] Nearest hospital:" << endl;
                    nearest->print();
                    cout << "\nDistance: " << fixed << setprecision(2) 
                         << minD << " units" << endl;
                    cout << "ETA: " << (minD / 15) << " minutes" << endl;
                }
                pause();
                break;
            }
            
            case 8: {  // Display Tree
                printDivider();
                cout << "\n--- QUADTREE STRUCTURE ---\n" << endl;
                qt->display();
                pause();
                break;
            }
            
            case 9: {  // Show All Locations
                BoundingBox all(100, 100, 100, 100);
                vector<Location> allLocs;
                qt->queryRange(all, allLocs);
                
                cout << "\n--- ALL LOCATIONS ---" << endl;
                cout << "Total: " << allLocs.size() << endl << endl;
                
                for (size_t i = 0; i < allLocs.size(); i++) {
                    allLocs[i].print();
                    cout << endl;
                }
                pause();
                break;
            }
            
            case 10: {  // Load Sample Data
                loadSampleData(qt, nextId);
                cout << "\n[SUCCESS] Sample data loaded!" << endl;
                cout << "- 5 Cabs for ride-sharing demo" << endl;
                cout << "- 4 Restaurants" << endl;
                cout << "- 3 Hospitals for emergency" << endl;
                cout << "- 4 User locations" << endl;
                pause();
                break;
            }
            
            case 11: {  // Statistics
                printDivider();
                cout << "\n--- STATISTICS ---\n" << endl;
                cout << "Total Locations: " << qt->countLocations() << endl;
                
                vector<Location> cabs, rest, hosp, users;
                qt->getLocationsByType("CAB", cabs);
                qt->getLocationsByType("RESTAURANT", rest);
                qt->getLocationsByType("HOSPITAL", hosp);
                qt->getLocationsByType("USER", users);
                
                cout << "  Cabs: " << cabs.size() << endl;
                cout << "  Restaurants: " << rest.size() << endl;
                cout << "  Hospitals: " << hosp.size() << endl;
                cout << "  Users: " << users.size() << endl;
                cout << "\nCoverage: 200 x 200 units" << endl;
                cout << "Node Capacity: 4 locations" << endl;
                pause();
                break;
            }
            
            case 0: {  // Exit
                running = false;
                cout << "\nThank you for using Spatial Indexing System!" << endl;
                break;
            }
            
            default:
                cout << "\n[ERROR] Invalid choice!" << endl;
                pause();
        }
    }
    
    delete qt;
    return 0;
}