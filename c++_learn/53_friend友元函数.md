```cpp
class Node { public: template <typename M0, typename M1, typename M2, typename M3>
friend class Component; 
friend class TimerComponent; 
friend bool Init(const char*, const std::string&); 
friend std::unique_ptr<Node> CreateNode(const std::string&, const std::string&);
```