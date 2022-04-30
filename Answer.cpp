#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
	explicit Vect(int x, int y) : x(x), y(y) {}		// BE CAREFUL TO CALL THE RIGHT CONSTRUCTOR (int, int) or (int, double)
	Vect(int norm, double dir) : x(norm*cos(dir)), y(norm*sin(dir)) { cerr << dir << endl; }	//	dir is in radians, call to_rad(double) to use degrees.
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
	const int	health() const { return _health; }
	const int	mana() const { return _mana; }

private:
	int _health; // Each player's base health
	int _mana;   // Ignore in the first league; Spend ten mana to cast a spell

	friend istream& operator>>(istream& in, Player& rhs)
	{
		in >> rhs._health >> rhs._mana;
		return in;
	}
};

struct Base
{
	const Point&	xy() const { return _xy; }
	const Point&	adv() const { return _adv; }

private:
	Point _xy;
	Point _adv;

	friend istream& operator>>(istream& in, Base& rhs)
	{
		in >> rhs._xy;
		rhs._adv = P_MAX - rhs._xy;
		return in;
	}
};

struct Entity
{
	Entity() {}
	Entity(const Entity& instance) { *this = instance; }
	Entity&	operator=(const Entity& rhs)
	{
		id = rhs.id;
		type = rhs.type;
		xy = rhs.xy;
		shield_life = rhs.shield_life;
		is_controlled = rhs.is_controlled;
		health = rhs.health;
		vxy = rhs.vxy;
		near_base = rhs.near_base;
		base_dist = rhs.base_dist;
		adv_dist = rhs.adv_dist;
		return *this;
	}

	int id;				// Unique identifier
	int type;			// 0=monster, 1=your hero, 2=opponent hero
	Point xy;			// Position point of this entity
	int shield_life;	// Count down until shield spell fades
	int is_controlled;	// Equals 1 when this entity is under a control spell
	int health;			// Remaining health of this monster
	Vect vxy;			// Trajectory vector of this monster
	int near_base;		// 0=monster with no target yet, 1=monster targeting a base
	int threat_for;		// Given this monster's trajectory, is it a threat to 1=your base, 2=your opponent's base, 0=neither
	int base_dist;		// Distance to base.
	int adv_dist;		// Distance to adverse base.
	int	hero_dist[3];	// Distance to each hero.

	friend istream& operator>>(istream& in, Entity& rhs)
	{
		in >> rhs.id >> rhs.type >> rhs.xy.x >> rhs.xy.y >> rhs.shield_life >> rhs.is_controlled;
		in >> rhs.health >> rhs.vxy.x >> rhs.vxy.y >> rhs.near_base >> rhs.threat_for;
		return in;
	}
};

int	main()
{
	Player	me;
	Player	adv;

	Base	base;
	const int&	base_x = base.xy().x;
	const int&	base_y = base.xy().y;
	cin >> base; cin.ignore();

	int		heroes_per_player;
	cin >> heroes_per_player; cin.ignore();

	vector<Entity>						heroes(3);
	vector<Entity>						enemies(3);
	vector<Entity>						monsters;
	monsters.reserve(300);

	multimap<int, Entity*>				baseview;
//	vector<multimap<int, Entity*> >		heroviews(3);	// DEACTIVATED, TOO RESSOURCE CONSUMING

	clock_t clk_loop_start;

	// game loop
	while (1)
	{
		clk_loop_start = clock();

		monsters.clear();
		baseview.clear();
/*		heroviews[0].clear();	// DEACTIVATED, TOO RESSOURCE CONSUMING
		heroviews[1].clear();
		heroviews[2].clear();*/

		cin >> me; cin.ignore();
		cin >> adv;	cin.ignore();

		const int	health = me.health();
		const int	mana = me.mana();

		int entity_count; // Amount of heros and monster you can see
		cin >> entity_count; cin.ignore();

		cerr << "Entity count : " << entity_count << endl;

		for (int i = 0; i < entity_count; i++)
		{
			Entity tmp;
			cin >> tmp; cin.ignore();

			tmp.base_dist = base.xy().dist(tmp.xy);
			tmp.adv_dist = base.adv().dist(tmp.xy);
			for (int i = 0; i < heroes_per_player; i++)
				if (tmp.id != heroes[i].id)
					tmp.hero_dist[i] = heroes[i].xy.dist(tmp.xy);
				else
					tmp.hero_dist[i] = 0;

			switch (tmp.type)
			{
			case 0: 	//	Monsters
				{
					//	Store into monster vector.
					monsters.push_back(tmp);

					//	Stores the pointer into useful map tables.
					Entity*	ptr = &monsters.back();

					//	monsters sorted by distance view from base.
					if (ptr->base_dist < 10000)
						baseview.insert(make_pair(ptr->base_dist, ptr));

					//	monsters at distance view from each hero.		// DEACTIVATED, TOO RESSOURCE CONSUMING
					/*for (int i = 0; i < heroes.size(); i++)
						if (ptr->hero_dist[i] < 2200)
							heroviews[i].insert(make_pair(ptr->hero_dist[i], ptr));*/
				}
				break;
			case 1:		//	Heroes
				heroes.push_back(tmp);
				break;
			case 2:		//	Enemies
				enemies.push_back(tmp);
				break;
			}
		}

		auto monsterit = baseview.begin();

		for (int i = 0; i < heroes_per_player; i++)
		{
			// DO YOUR CODE ! :-)

			if (monsterit != baseview.end())
			{
				Point p = monsterit->second->xy + monsterit->second->vxy;
				cout << "MOVE " << p << " kill " << monsterit->second->id << endl;
				++monsterit;
			}
			else
			{
				cout << "MOVE " << Point(0, 0) + Vect(7000, to_rad(22.5 * (i+1))) << " post " << i << endl;
			}

		}

		cerr << "Turn exec_time (in clock ticks) : " << clock() - clk_loop_start << endl;
	}
}
