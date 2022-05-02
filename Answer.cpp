#include <algorithm>
#include <cmath>
#include <ctime>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>

#define X_MAX 17630
#define Y_MAX 9000
#define DEFAULT_SAME_DIR_MAX_ANGLE_DEG 20
#define VECT_NORM 400

using namespace std;

double	to_rad(const double deg) { return deg * M_PI / 180.; }
double	to_deg(const double rad) { return rad * 180. / M_PI; }

class Point
{
public:
	Point() : x(0), y(0) {}
	Point(const int x, const int y) : x(x), y(y) {}
	Point(const Point& instance) : x(instance.x), y(instance.y) {}

	Point&			operator=(const Point& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		return *this;
	}

	int x;
	int y;

	Point			operator+(const Point& p) const { return Point(x + p.x, y + p.y); }
	Point			operator-(const Point& p) const { return Point(x - p.x, y - p.y); }
	Point			operator-() const { return Point(-x, -y); }
	Point			operator*(int n) const { return Point(x * n, y * n); }
	Point			operator/(int n) const { return Point(x / n, y / n); }

	bool			operator==(const Point& p) const { return x == p.x && y == p.y; }
	bool			operator!=(const Point& p) const { return	!(*this == p); }

	Point			mid(const Point& b) const { return (*this + b)/ 2; }
	friend Point	mid(const Point& a, const Point& b) { return a.mid(b); }

	Point			sym(const Point& ref) const { return ref * 2 - *this; }
	friend Point	sym(const Point& p, const Point& ref) { return p.sym(ref); }

	Point			abs() const { return Point(std::abs(x), std::abs(y)); }
	friend Point	abs(const Point& p) { return p.abs(); }

	int				dist(const Point& b) const
	{
		int dx = std::abs(x - b.x);
		int dy = std::abs(y - b.y);
		return std::sqrt(dx * dx + dy * dy);
	}
	friend int		dist(const Point& a, const Point& b) { return a.dist(b); }

	friend ostream& operator<<(ostream& out, const Point& rhs)
	{
		out << rhs.x << " " << rhs.y;
		return out;
	}
	friend istream& operator>>(istream& in, Point& rhs)
	{
		in >> rhs.x >> rhs.y;
		return in;
	}
};

//  Point constants
static const Point	P_ZERO(0, 0);        // (0,0)
static const Point	P_MAX(X_MAX, Y_MAX); // ()
static const Point	P_MID(X_MAX / 2, Y_MAX / 2);

struct Vect
{
	Vect() : x(), y() {}
	Vect(int x, int y) : x(x), y(y) {}		// BE CAREFUL TO CALL THE RIGHT CONSTRUCTOR (int, int) or (int, double)
	Vect(int norm, double dir) : x(norm*cos(dir)), y(norm*sin(dir)) {}	//	dir is in radians, call to_rad(double) to use degrees.
	Vect(const Point& origin, const Point& dest) : x(dest.x-origin.x), y(dest.y-origin.y) {}
	Vect(const Vect& instance) : x(instance.x), y(instance.y) {}

	Vect&			operator=(const Vect& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		return *this;
	}

	int x;
	int y;

	Vect			operator+(const Vect& v) const { return Vect(x + v.x, y + v.y); }
	Vect			operator-(const Vect& v) const { return Vect(x - v.x, y - v.y); }
	Vect			operator-() const { return Vect(-x, -y); }
	Vect			operator*(int n) const { return Vect(x * n, y * n); }
	Vect			operator/(int n) const { return Vect(x / n, y / n); }

	bool			operator==(const Vect& v) const { return x == v.x && y == v.y; }
	bool			operator!=(const Vect& v) const { return	!(*this == v); }

	double			dir() const { return atan2(y, x); } //	Direction of a vector [-Pi, Pi]
	friend double	dir(const Vect& v) { return v.dir(); }
	double			angle(const Vect& v) const
	{
		return	abs(dir() - v.dir());
	} //	Absolute minimum angle between two vectors. [0, Pi]
	friend double	angle(const Vect& v1, const Vect& v2) { return v1.angle(v2); }

	// Norm (length) of vector ( But all monsters vxy norm is 400 for info )
	double			norm() const
	{
		int dx = std::abs(x);
		int dy = std::abs(y);
		return std::sqrt(dx * dx + dy * dy);
	}
	friend double	norm(const Vect& v) { return v.norm(); }
	Vect			normalize() const { return Vect(100, dir()); }
	friend Vect		normalize(const Vect& v) { return v.normalize(); }

	// Scalar product (is > 0 if angle > 90 degrees)
	int				scal_prod(const Vect& v) const { return x * v.x + y * v.y; }
	friend int		scal_prod(const Vect& v1, const Vect& v2)
	{
		return v1.scal_prod(v2);
	}

	// are vectors going approximately same direction ? (angle <= max_angle)
	bool			same_dir(const Vect& v, double max_angle) const
	{
		return	angle(v) <= max_angle;
	}
	friend bool		same_dir(const Vect& v1, const Vect& v2, double max_angle)
	{
		return v1.same_dir(v2, max_angle);
	}
	// simplified version using DEFAULT_SAME_DIR_MAX_ANGLE_DEG
	bool			same_dir(const Vect& v) const
	{
		static const double MAX_ANGLE_RAD = to_rad(DEFAULT_SAME_DIR_MAX_ANGLE_DEG);
		return		angle(v) <= MAX_ANGLE_RAD;
	}
	friend bool		same_dir(const Vect& v1, const Vect& v2)
	{
		return v1.same_dir(v2);
	}

	friend istream& operator>>(istream& in, Vect& rhs)
	{
		in >> rhs.x >> rhs.y;
		return in;
	}
};

//	For retrieving a destination point by adding a point and a vector.
Point			operator+(const Point& p, const Vect& v) { return Point(p.x+v.x, p.y+v.y); }

struct Player
{
	int health; // Each player's base health
	int mana;   // Ignore in the first league; Spend ten mana to cast a spell

	friend istream& operator>>(istream& in, Player& rhs)
	{
		in >> rhs.health >> rhs.mana;
		return in;
	}
};

struct Base
{
	Point xy;
	Point adv;
	std::vector<Point> posts;

	Point get_post(int i)
	{
		posts.resize(3);
		if (xy.x)
			return adv-posts[i];
		return posts[i];
	}

	friend istream& operator>>(istream& in, Base& rhs)
	{
		in >> rhs.xy;
		rhs.adv = P_MAX - rhs.xy;
		return in;
	}
};

struct Entity
{
	int id;				// Unique identifier
	int type;			// 0=monster, 1=your hero, 2=opponent hero
	Point xy;			// Position point of this entity
	int shield_life;	// Count down until shield spell fades
	int is_controlled;	// Equals 1 when this entity is under a control spell
	int health;			// Remaining health of this monster
	Vect vxy;			// Trajectory vector of this monster
	Point dst;			// Destination point for next turn
	int near_base;		// 0=monster with no target yet, 1=monster targeting a base
	int threat_for;		// Given this monster's trajectory, is it a threat to 1=your base, 2=your opponent's base, 0=neither

	void	displace(const Vect& v) { xy = xy+v; dst = dst+v; }

	friend istream& operator>>(istream& in, Entity& rhs)
	{
		in >> rhs.id >> rhs.type >> rhs.xy.x >> rhs.xy.y >> rhs.shield_life >> rhs.is_controlled;
		in >> rhs.health >> rhs.vxy.x >> rhs.vxy.y >> rhs.near_base >> rhs.threat_for;
		rhs.dst = rhs.xy + rhs.vxy;
		return in;
	}
};

static const std::map<std::string, int Entity::*> EntityIntMembers = {
	{"id", &Entity::id},
	{"type", &Entity::type},
	{"shield_life", &Entity::shield_life},
	{"is_controlled", &Entity::is_controlled},
	{"health", &Entity::health},
	{"near_base", &Entity::near_base},
	{"threat_for", &Entity::threat_for}
};

class EntityCompare		// Gereric virtual binary predicate on entities or pointer on entities.
{
public:
	virtual bool	operator()(const Entity& a, const Entity& b) const final { return compared_value(a) < compared_value(b); };
	virtual	bool	operator()(const Entity* a, const Entity* b) const final { return compared_value(*a) < compared_value(*b); };
	virtual	int		compared_value(const Entity& e) const { return e.id; }
};

class EntityMemberCompare : public EntityCompare	// Binary predicate comparing int members of two entities.
{
public:
	EntityMemberCompare(const std::string& member) {
		auto f = EntityIntMembers.find(member);
		if (f != EntityIntMembers.end())
			_ptr_m = f->second;
		else
			_ptr_m = &Entity::id;
	}
	virtual	int		compared_value(const Entity& e) const { return e.*_ptr_m; }
private:
	int Entity::* _ptr_m;
};

class EntityDistCompare : public EntityCompare		// Binary predicate comparing entities distance to a given reference point.
{
public:
	EntityDistCompare(const Point& ref_point) : p(ref_point) {}
	EntityDistCompare(Point&& ref_point) : p(std::move(ref_point)) {}
	virtual	int		compared_value(const Entity& e) const { return e.xy.dist(p); }
private:
	const Point p;
};

class EntityDestCompare : public EntityCompare		// Binary predicate comparing entities distance to a given reference point.
{
public:
	EntityDestCompare(const Point& ref_point) : p(ref_point) {}
	EntityDestCompare(Point&& ref_point) : p(std::move(ref_point)) {}
	virtual	int		compared_value(const Entity& e) const { return e.dst.dist(p); }
private:
	const Point p;
};

class EntityAngleCompare : public EntityCompare		// Binary predicate comparing entities direction vector angle to a to a given reference vector.
{
public:
	EntityAngleCompare(const Vect& ref_vect) : v(ref_vect) {}
	EntityAngleCompare(Vect&& ref_vect) : v(std::move(ref_vect)) {}
	virtual	int		compared_value(const Entity& e) const { return e.vxy.angle(v); }
private:
	const Vect v;
};

class EntitySelect		// Gereric virtual unary predicate on entities or pointer on entities.
{
public:
	EntitySelect(int max) : _mode(0), _max(max) {}
	EntitySelect(int min, int max) : _mode(1), _min(min), _max(max) {}
	EntitySelect(const std::initializer_list<int>& values) : _mode(2), _values(values) {}
	EntitySelect(std::initializer_list<int>&& values) : _mode(2), _values(std::move(values)) {}
	virtual	int		selected_value(const Entity& e) const = 0;

	virtual bool	operator()(const Entity* e) const final { return (*this)(*e); }
	virtual bool	operator()(const Entity& e) const final
	{
		switch(_mode)
		{
			case 0:		return selected_value(e) <= _max;
			case 1:		return _min <= selected_value(e) && selected_value(e) <= _max;
			case 2:		return std::find(_values.begin(), _values.end(), selected_value(e)) != _values.end();
			default:	return false;
		}
	}
private:
	int							_mode;
	std::initializer_list<int>	_values;
	int 						_min;
	int 						_max;
};

class EntityMemberSelect : public EntitySelect		// Unnary predicate selecting entities on a member value.
{
public:
	EntityMemberSelect(const std::string& member, int max) : _ptr_m(_init_ptr(member)), EntitySelect(max) {}
	EntityMemberSelect(const std::string& member, int min, int max) : _ptr_m(_init_ptr(member)), EntitySelect(min, max) {}
	EntityMemberSelect(const std::string& member, const std::initializer_list<int>& values) : _ptr_m(_init_ptr(member)), EntitySelect(values) {}
	EntityMemberSelect(const std::string& member, std::initializer_list<int>&& values) : _ptr_m(_init_ptr(member)), EntitySelect(std::move(values)) {}
	virtual	int				selected_value(const Entity& e) const { return e.*_ptr_m; }
private:
	static int Entity::*	_init_ptr(const std::string& member)
	{
		auto f = EntityIntMembers.find(member);
		if (f != EntityIntMembers.end())
			return f->second;
		else
			return &Entity::id;
	}
	int Entity::*			_ptr_m;
};

class EntityDistSelect : public EntitySelect		// Unnary predicate selecting entities on a distance to a given reference point.
{
public:
	EntityDistSelect(const Point& ref_point, int max) : _ref(ref_point), EntitySelect(max) {}
	EntityDistSelect(const Point& ref_point, int min, int max) : _ref(ref_point), EntitySelect(min, max) {}
	EntityDistSelect(const Point& ref_point, const std::initializer_list<int>& values) : _ref(ref_point), EntitySelect(values) {}
	EntityDistSelect(const Point& ref_point, std::initializer_list<int>&& values) : _ref(ref_point), EntitySelect(std::move(values)) {}
	EntityDistSelect(Point&& ref_point, int max) : _ref(std::move(ref_point)), EntitySelect(max) {}
	EntityDistSelect(Point&& ref_point, int min, int max) : _ref(std::move(ref_point)), EntitySelect(min, max) {}
	EntityDistSelect(Point&& ref_point, const std::initializer_list<int>& values) : _ref(std::move(ref_point)), EntitySelect(values) {}
	EntityDistSelect(Point&& ref_point, std::initializer_list<int>&& values) : _ref(std::move(ref_point)), EntitySelect(std::move(values)) {}
	virtual	int				selected_value(const Entity& e) const { return e.xy.dist(_ref); }
private:
	const Point				_ref;
};

class EntityDestSelect : public EntitySelect		// Unnary predicate selecting entities on a distance to a given reference point.
{
public:
	EntityDestSelect(const Point& ref_point, int max) : _ref(ref_point), EntitySelect(max) {}
	EntityDestSelect(const Point& ref_point, int min, int max) : _ref(ref_point), EntitySelect(min, max) {}
	EntityDestSelect(const Point& ref_point, const std::initializer_list<int>& values) : _ref(ref_point), EntitySelect(values) {}
	EntityDestSelect(const Point& ref_point, std::initializer_list<int>&& values) : _ref(ref_point), EntitySelect(std::move(values)) {}
	EntityDestSelect(Point&& ref_point, int max) : _ref(std::move(ref_point)), EntitySelect(max) {}
	EntityDestSelect(Point&& ref_point, int min, int max) : _ref(std::move(ref_point)), EntitySelect(min, max) {}
	EntityDestSelect(Point&& ref_point, const std::initializer_list<int>& values) : _ref(std::move(ref_point)), EntitySelect(values) {}
	EntityDestSelect(Point&& ref_point, std::initializer_list<int>&& values) : _ref(std::move(ref_point)), EntitySelect(std::move(values)) {}
	virtual	int				selected_value(const Entity& e) const { return e.dst.dist(_ref); }
private:
	const Point				_ref;
};

class EntityAngleSelect : public EntitySelect		// Unnary predicate selecting entities on an angle to a given reference vector.
{
public:
	EntityAngleSelect(const Vect& ref_vect, int max) : _ref(ref_vect), EntitySelect(max) {}
	EntityAngleSelect(const Vect& ref_vect, int min, int max) : _ref(ref_vect), EntitySelect(min, max) {}
	EntityAngleSelect(const Vect& ref_vect, const std::initializer_list<int>& values) : _ref(ref_vect), EntitySelect(values) {}
	EntityAngleSelect(const Vect& ref_vect, std::initializer_list<int>&& values) : _ref(ref_vect), EntitySelect(std::move(values)) {}
	EntityAngleSelect(Vect&& ref_vect, int max) : _ref(std::move(ref_vect)), EntitySelect(max) {}
	EntityAngleSelect(Vect&& ref_vect, int min, int max) : _ref(std::move(ref_vect)), EntitySelect(min, max) {}
	EntityAngleSelect(Vect&& ref_vect, const std::initializer_list<int>& values) : _ref(std::move(ref_vect)), EntitySelect(values) {}
	EntityAngleSelect(Vect&& ref_vect, std::initializer_list<int>&& values) : _ref(std::move(ref_vect)), EntitySelect(std::move(values)) {}
	virtual	int				selected_value(const Entity& e) const { return e.vxy.angle(_ref); }
private:
	const Vect				_ref;
};

//	Utility class for remapping entities pointer including sorting and selection.
class Remap
{
public:
	typedef multiset<Entity*, EntityCompare>		EntitySet;

	template<typename... Containers>
	static EntitySet	create_set(const EntityCompare& cmp, Containers&... src_containers)
	{
		EntitySet	set(cmp);
		add_to_set(set, src_containers...);
		return set;
	}
	template<typename... Containers>
	static EntitySet	create_set(const EntityCompare& cmp, const EntitySelect& sel, Containers&... src_containers)
	{
		EntitySet	set(cmp);
		add_to_set(set, sel, src_containers...);
		return set;
	}
	template<typename... Containers>
	static void add_to_set(EntitySet& set, Containers&... src_containers)
	{
		(void)std::initializer_list<int>{(_int_add_to_set(set, src_containers), 0)...};
	}
	template<typename... Containers>
	static void add_to_set(EntitySet& set, const EntitySelect& sel, Containers&... src_containers)
	{
		(void)std::initializer_list<int>{(_int_add_to_set(set, sel, src_containers), 0)...};
	}
	static int	compared_value(const EntitySet& set, const Entity& e) { return set.key_comp().compared_value(e); }
	static int	compared_value(const EntitySet& set, const EntitySet::iterator& it) { return set.key_comp().compared_value(**it); }
private:
	template<typename Container>
	static void _int_add_to_set(EntitySet& set, Container& src_container)
	{
		for (auto it = src_container.begin(); it != src_container.end(); ++it)
			set.insert(_get_entity_ptr(it));
	}
	template<typename Container>
	static void _int_add_to_set(EntitySet& set, const EntitySelect& sel, Container& src_container)
	{
		for (auto it = src_container.begin(); it != src_container.end(); ++it)
		{
			Entity* ptr = _get_entity_ptr(it);
			if (sel(ptr))
				set.insert(ptr);
		}
	}
	template<typename T, typename... Args>
	static bool		_and(T first, Args... rest) { return first && _and(rest...); }
	static bool		_and(bool single) { return single; }
	static Entity*	_get_entity_ptr(const std::vector<Entity>::iterator& it) { return &*it; }
	static Entity*	_get_entity_ptr(const std::multiset<Entity*>::iterator& it) { return *it; }
};

void	wind_entities(const Remap::EntitySet& windport, Vect wind_dir, int* mana)
{
	*mana -= 10;
	for (auto it = windport.begin(); it != windport.end(); ++it)
		if (!(*it)->shield_life)
			(*it)->displace(Vect(2200, wind_dir.dir()));
}

int	main()
{
	Player	me;
	Player	adv;

	Base	base;
	const int&	base_x = base.xy.x;
	const int&	base_y = base.xy.y;
	cin >> base; cin.ignore();

	int		heroes_per_player;
	cin >> heroes_per_player; cin.ignore();

	//	Create and pre-allocates entities storage vectors.
	vector<Entity>	heroes;
	vector<Entity>	enemies;
	vector<Entity>	monsters;
	heroes.reserve(3);
	enemies.reserve(3);
	monsters.reserve(100);

	clock_t clk_loop_start;

	// game loop
	while (1)
	{
		clk_loop_start = clock();

		//	Clear all entities before reparse.
		heroes.clear();
		enemies.clear();
		monsters.clear();

		cin >> me; cin.ignore();
		cin >> adv;	cin.ignore();

		int&	health = me.health;
		int&	mana = me.mana;

		int entity_count; // Amount of heros and monster you can see
		cin >> entity_count; cin.ignore();

		cerr << "Entity count : " << entity_count << endl;

		monsters.reserve(entity_count - 3);

		for (int i = 0; i < entity_count; i++)
		{
			Entity tmp;
			cin >> tmp; cin.ignore();
			switch(tmp.type)
			{
				case 2:	enemies.push_back(std::move(tmp)); break;
				case 1:	heroes.push_back(std::move(tmp)); break;
				case 0:	monsters.push_back(std::move(tmp)); break;
			}
		}

		base.posts = {Point(0,0)+Vect(6000, to_rad(15)), Point(0,0)+Vect(6000, to_rad(40)), Point(0,0)+Vect(6000, to_rad(65))};

		///	Hero 0		(Defense)
		for (int i = 0; i < heroes_per_player; ++i)
		{
			{
				auto	base_view(Remap::create_set(EntityDistCompare(base.xy), monsters, enemies));
				auto	base_threats(Remap::create_set(EntityDestCompare(base.xy), EntityDestSelect(base.xy, 5500), base_view));
				auto	viewport(Remap::create_set(EntityDistCompare(heroes[i].xy), EntityDistSelect(heroes[i].xy, 2200), base_view));
				auto	windport(Remap::create_set(EntityDistCompare(heroes[i].xy), EntityDistSelect(heroes[i].xy, 1280), viewport));
				auto	view_threats(Remap::create_set(EntityDistCompare(base.xy), EntityMemberSelect("threat_for", {1}), viewport));
				cerr << "view[" << i << "] : " << viewport.size() << endl;
				if (windport.size() >= 2 && mana >= 10 && Remap::create_set(EntityDistCompare(base.xy), EntityDistSelect(heroes[i].xy, 6000), windport).size())
				{
					Vect	dir = Vect(base.xy, heroes[i].xy).normalize();
					cout << "SPELL WIND " << heroes[i].xy + dir << " URG" << endl;
					wind_entities(windport, dir, &mana);
					continue;
				}
				if (base_threats.size())
				{
					cout << "MOVE " << (*base_threats.begin())->dst << " kill " << i << endl;
					continue;
				}
				if (mana >= 70 && view_threats.size())
				{
					cout << "SPELL CONTROL " << (*view_threats.begin())->id << " " << base.adv << " wololo"<< endl;
					continue;
				}
				cout << "MOVE " << base.get_post(i) << " post " << i << endl;
				continue;
			}
		}

		cerr << "Turn exec_time (in clock ticks) : " << clock() - clk_loop_start << endl;
	}
}
