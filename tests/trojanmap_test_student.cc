#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, Test1) {
  EXPECT_EQ(true, true);
}

// Phase 1

// Test GetLat() function
TEST(TrojanMapTest, GetLat) {
  TrojanMap m;
  // Exist id
  EXPECT_EQ(m.GetLat("653725"), 34.0360852);
  // Non-exist id
  EXPECT_EQ(m.GetLat("100"), -1);
}

// Test GetLon() function
TEST(TrojanMapTest, GetLon) {
  TrojanMap m;
  // Exist id
  EXPECT_EQ(m.GetLon("653725"), -118.3212048);
  // Non-exist id
  EXPECT_EQ(m.GetLon("100"), -1);
}

// Test GetName() function
TEST(TrojanMapTest, GetName) {
  TrojanMap m;
  // Exist id, no name
  EXPECT_EQ(m.GetName("653725"), "");
  // Exist id, exist name
  EXPECT_EQ(m.GetName("368167117"), "Ahmanson Commons");
  // Non-exist id
  EXPECT_EQ(m.GetName("100"), "NULL");
}

// Test GetNeighborIDs() function
TEST(TrojanMapTest, GetNeighborIDs) {
  TrojanMap m;
  // Exist id
  std::vector<std::string> expected_result_1 = {"277327731", "1613324102"};
  EXPECT_EQ(m.GetNeighborIDs("653725"), expected_result_1);
  // Exist id
  std::vector<std::string> expected_result_2 = {"6816288727"};
  EXPECT_EQ(m.GetNeighborIDs("368167117"), expected_result_2);
  // Non-exist id
  std::vector<std::string> expected_result_3 = {};
  EXPECT_EQ(m.GetNeighborIDs("100"), expected_result_3);
}

// Test GetID Function
TEST(TrojanMapTest, GetID) {
  TrojanMap m;
  EXPECT_EQ(m.GetID("Ahmanson Commons"), "368167117");
  EXPECT_EQ(m.GetID("Chipotle"), "732641023");
  EXPECT_EQ(m.GetID("No place"), "");
}

// Test Autocomplete function
TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Usc");
  std::unordered_set<std::string> gt = {"USC Village Gym", "USC Parking", "USC Fisher Museum of Art", "USC Roski Eye Institute", "USC Credit Union"}; // groundtruth for "USC"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("usc");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("uSc"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("USC"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  
  // Test USC Village Gym
  auto position = m.GetPosition("USC Village Gym");
  std::pair<double, double> gt1(34.0252392, -118.2858186); // groundtruth for "USC Village Gym"
  EXPECT_EQ(position, gt1);
  // Test USC Parking
  position = m.GetPosition("USC Parking");
  std::pair<double, double> gt2(34.0238824, -118.2801114); // groundtruth for "USC Parking"
  EXPECT_EQ(position, gt2);
  // Test USC Fisher Museum of Art
  position = m.GetPosition("USC Fisher Museum of Art");
  std::pair<double, double> gt3(34.0186092, -118.2873476); // groundtruth for "USC Fisher Museum of Art"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("abcdef", "bcf"), 3);
  EXPECT_EQ(m.CalculateEditDistance("leaveylib", "dohenylib"), 5);
}

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Starbks"), "Starbucks");
  EXPECT_EQ(m.FindClosestName("Dulllce"), "Dulce");
}

// Test ReadLocationsFromCSVFile function
TEST(TrojanMapTest, ReadLocationsFromCSVFile) {
  TrojanMap m;

  // Input path must be changed depend on location on different computer
  std::string input_file_name = "/home/ee538/Documents/final-project-youungjulie/input/topologicalsort_locations.csv";
  std::vector<std::string> expected_output = {"Ralphs", "KFC", "Chick-fil-A"};
  EXPECT_EQ(m.ReadLocationsFromCSVFile(input_file_name), expected_output);
}

// Test ReadDependenciesFromCSVFile function
TEST(TrojanMapTest, ReadDependenciesFromCSVFile) {
  TrojanMap m;

  // Input path must be changed depend on location on different computer
  std::string input_file_name = "/home/ee538/Documents/final-project-youungjulie/input/topologicalsort_dependencies.csv";
  std::vector<std::vector<std::string>> expected_output = {{"Ralphs","Chick-fil-A"}, {"Ralphs","KFC"}, {"Chick-fil-A", "KFC"}};
  EXPECT_EQ(m.ReadDependenciesFromCSVFile(input_file_name), expected_output);
}
 
TEST(TrojanMapTest, DeliveringTrojan_1) {
  TrojanMap m;
  std::vector<std::string> locations = {"Ralphs", "KFC", "Chick-fil-A"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","Chick-fil-A"}, {"Ralphs","KFC"}, {"Chick-fil-A", "KFC"}};
  std::vector<std::string> expected_result = {"Ralphs", "Chick-fil-A", "KFC"};
  EXPECT_EQ(m.DeliveringTrojan(locations, dependencies), expected_result);
}

TEST(TrojanMapTest, DeliveringTrojan_cycle) {
  TrojanMap m;
  std::vector<std::string> locations = {"Ralphs", "KFC", "Chick-fil-A"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"KFC", "Chick-fil-A"}, {"Chick-fil-A", "Ralphs"}};
  std::vector<std::string> expected_result = {};
  EXPECT_EQ(m.DeliveringTrojan(locations, dependencies), expected_result);
}