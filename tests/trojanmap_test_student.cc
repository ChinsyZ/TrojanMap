#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, Test1) {
  EXPECT_EQ(true, true);
}

// Phase 1
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

