#pragma once
#include <functional>
extern "C" uint32_t micro_timestamp; // needs to be updated somehow on each frame (helps with line/intersection tracking time based validation)

#define CAM_RES0                0x00 
#define CAM_RES1                0x01
#define CAM_RES2                0x02
#define CAM_RES0_WIDTH          1280
#define CAM_RES0_HEIGHT         800
#define CAM_RES1_WIDTH          640
#define CAM_RES1_HEIGHT         400
#define CAM_RES2_WIDTH          316
#define CAM_RES2_HEIGHT         208
#define CAM_RES3_WIDTH          (CAM_RES2_WIDTH+320) //636
#define CAM_RES3_HEIGHT         (CAM_RES2_HEIGHT/2) //104

#define SRAM1_LOC                smram1///0x10080000
#define SRAM1_SIZE               0x12000 //73728 > 636*104+64+636=66144+64+636=66208+636=66844
#define SRAM4_LOC                smram4///0x2000c000
#define SRAM4_SIZE               0x4000
extern uint8_t SRAM1_LOC[];
extern uint8_t SRAM4_LOC[];


#define CAM_PREBUF_LEN			    64
#define CAM_M0R3_PTR (SRAM1_LOC+CAM_PREBUF_LEN)
#define CAM_M0R3_LEN (CAM_RES3_WIDTH*CAM_RES3_HEIGHT)

//typedef void (*line_event)(uint8_t idx, uint8_t m_x0, uint8_t m_y0, uint8_t m_x1, uint8_t m_y1);
//extern line_event line_detected;
using line_detect_event_t = std::function<void(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t)>;
extern line_detect_event_t line_detected;
extern line_detect_event_t vector_detected;
//using line_detect_event_t = std::function<void(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t)>;
using intersection_detect_event_t = std::function<void(uint8_t degree, uint8_t x, uint8_t y)>;
extern intersection_detect_event_t intersection_detected;
//extern std::function line_detected;
//std::function<void(uint8_t idx, uint8_t m_x0, uint8_t m_y0, uint8_t m_x1, uint8_t m_y1)> line_detected;

struct EqueueFields
{
    volatile uint16_t readIndex;
    volatile uint16_t writeIndex;

    volatile uint16_t produced;
    volatile uint16_t consumed;

    // (array size below doesn't matter-- we're just going to cast a pointer to this struct)
    uint16_t data[1]; // data
};

typedef int16_t LineSegIndex;

struct Point
{
	Point()
	{
	}
	
	Point(uint8_t x, uint8_t y)
	{
		m_x = x;
		m_y = y;
	}
	
	bool equals(const Point &p) const
	{
		return p.m_x==m_x && p.m_y==m_y;
	}
	
	uint16_t dist2(const Point &p) const
	{
		int16_t diffx, diffy;
		
		diffx = p.m_x - m_x;
		diffy = p.m_y - m_y;
		
		return diffx*diffx + diffy*diffy;
	}
	
	void avg(const Point &p)
	{
		m_x = (m_x+p.m_x)>>1;
		m_y = (m_y+p.m_y)>>1;
	}
	
	uint8_t m_x;
	uint8_t m_y;
};

struct LineSeg
{
	void reset()
	{
		m_ls0 = m_ls1 = -1;
		m_line = 0;
	}
		
	uint16_t length2() const
	{
		int16_t diffx, diffy;
		
		diffx = m_p1.m_x - m_p0.m_x;
		diffy = m_p1.m_y - m_p0.m_y;
		
		return diffx*diffx + diffy*diffy;
	}

	Point m_p0; 
	Point m_p1;
	LineSegIndex m_ls0;
	LineSegIndex m_ls1;
	uint8_t m_line;
};

class Equeue
{
public:
	Equeue();
	~Equeue();

	void reset();
	uint32_t dequeue(uint16_t *val);
	uint32_t queued()
	{
		return m_fields->produced - m_fields->consumed;
	}

	uint32_t readLine(uint16_t *mem, uint32_t size, bool *eof, bool *error);
	void flush();

	EqueueFields *m_fields;
};

struct EqueueFields *loop2(uint8_t const *d);
int line_open();
void line_close();
uint8_t line_shadowCallback(const char *id, uint32_t val); //transmitting parameters in key value pairs (case sensitive keys)