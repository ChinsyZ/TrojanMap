# EE 538 Final Project Report

This project focus on create a map application of location around USC campus by implementing data structure and graph algorithm. 

## Data Structure

To create the map, every location in the map is represented by Node. Every node is connected using graph data structure. 

### Classes

**Node**

Attributes:  

id: A unique id assign to each point  
lat: Latitude of this location  
lon: Longitude of this location  
name: Name of this location  
neighbors: List of the ids of all neighbor points  
attributes: List of the attributes of the location  

```cpp
class Node {
  public:
    Node(){};
    Node(const Node &n){id = n.id; lat = n.lat; lon = n.lon; name = n.name; neighbors = n.neighbors; attributes = n.attributes;};
    std::string id;    // A unique id assign to each point
    double lat;        // Latitude
    double lon;        // Longitude
    std::string name;  // Name of the location. E.g. "Bank of America".
    std::vector<std::string> neighbors;  // List of the ids of all neighbor points.
    std::unordered_set<std::string> attributes;  // List of the attributes of the location.
};
```

**Trojan Map**

Attributes:  

data: A map stores the id of location as key and its Node as value

*Function below are implementations of the map*


## Phase 1 report  

**GetLat()**  
Time Complexity: O(1)  
Space Complexity: O(1)  
Get the Latitude of a Node given its id  

**GetLon()**  
Time Complexity: O(1)  
Space Complexity: O(1)  
Get the Longitude of a Node given its id

**GetName()**  
Time Complexity: O(n)  
Space Complexity: O(1)  
Get the name of a Node given its id

**GetID**  
Time Complexity: O(1)  
Space Complexity: O(1)  
Get the id given its name

**GetNeighborIDs**  
Time Complexity: O(1)  
Space Complexity: O(1)  
Get the neighbor ids of a Node

**GetPosition**  
Time Complexity: O(1)  
Space Complexity: O(1)  
Given a location name, return its latitude and longitude. If id does not exist, return (-1, -1)  

**CalculateEditDistance**
Time Complexity: O(n*m)  
Space Complexity: O(n*m)  
*n and m are the length of two words*  
Calculate edit distance between two location names, that is how many substitution, deletion, and addition to modify one word to another. 

**FindClosestName**
Time Complexity: `O(n*m*d)`  
Space Complexity: `O(n*m*d)`  
*n and m are the length of two words, d is the number of all data nodes*  
Iterate through the map and find the name with smallest edit distance.  

**Autocomplete**  
Time Complexity: O(n)  
Space Complexity: O(n)  
Returns a vector of names given a partial name.  
Run time testing:  
Input: `ch`  
Output: 11 matches  
Time taken by function: 8ms   

## Phase 2 report

**CalculateDistance**

**CalculatePathLength**

**CalculateShortestPath_Dijkstra**