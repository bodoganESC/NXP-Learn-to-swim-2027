#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include "simplelist.h"
#include "fw.hpp"
#include "misc.h"
#include <cmath>

#ifdef MAX
#undef MAX
#endif
#ifdef MIN
#undef MIN
#endif
#define MAX(a, b)  (a>b ? a : b)
#define MIN(a, b)  (a<b ? a : b)

#define M_PI 3.14159265358979323846 

void cprintf(uint32_t flags, const char *format, ...)
{
		char  buf[128];
    va_list args;
    va_start(args, format);
    printf(format, args);
		//vsprintf((char *)buf, (char const *)format, args);
    va_end(args);

	//CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('T','E','X','T')), HINT32(flags), HSTRING(buf));
}

///tracker.h sof

#define TR_DEFAULT_LEADING_THRESH      500 // milliseconds
#define TR_DEFAULT_TRAILING_THRESH     500 // milliseconds
#define TR_MAXVAL                      0xffffffff
#define TR_MAXAGE                      0xff

#define TR_EVENT_INVALIDATED           0x01
#define TR_EVENT_VALIDATED             0x02

enum TrackerState
{
	TR_VALID, 
	TR_TRAILING, // considered valid
	TR_LEADING,  // considered invalid
	TR_INVALID
};

//template <typename Object, bool validFunc(const Object &, const Object &)> struct Tracker
//template <typename Object, uint8_t leadingThresh, uint8_t trailingThresh> struct Tracker
template <typename Object> struct Tracker
{
	Tracker(uint16_t leadingThresh=TR_DEFAULT_LEADING_THRESH, uint16_t trailingThresh=TR_DEFAULT_TRAILING_THRESH)
	{
		setTiming(leadingThresh, trailingThresh);
		reset();
	}
	
	Tracker(Object object, uint8_t index, uint16_t leadingThresh=TR_DEFAULT_LEADING_THRESH, uint16_t trailingThresh=TR_DEFAULT_TRAILING_THRESH)
	{
		m_object = object;
		m_index = index;
		setTiming(leadingThresh, trailingThresh);
		reset();
	}
	
	void reset()
	{
		m_state = TR_LEADING; // start in leading state
		m_minVal = TR_MAXVAL;
		m_minObject = NULL;
		m_events = 0;
		m_eventsShadow = 0;
		m_age = 0;
		setTimerMs(&m_timer);
	}
	
	void setTiming(uint16_t leadingThresh, uint16_t trailingThresh)
	{
		m_leadingThresh = leadingThresh;
		m_trailingThresh = trailingThresh;
	}
	
	void resetMin()
	{
		m_minVal = TR_MAXVAL;
	}
	
	void setMin(Object *object, uint32_t val)
	{
		m_minObject = object;
		m_minVal = val;
	}

	bool swappable(uint32_t min, Object *out)
	{
		if (out->m_tracker==NULL) // no tracker, so we're good 
			return true;
		else if	(min>=out->m_tracker->m_minVal) // tracker exists, but we need to be better...
			return false;
		else if (m_state==TR_VALID || m_state==TR_TRAILING) // if we're going to swap, we're good if we're valid, regardless
			return true;
		else // we (this) is invalid
			return (out->m_tracker->m_state==TR_LEADING || out->m_tracker->m_state==TR_INVALID); // if they are invalid, we're good to swap also
	}
	
	uint8_t update()
	{	
		uint8_t events = 0;
		bool success = m_minVal!=TR_MAXVAL;

		if (success)
			m_object = *m_minObject;
		switch(m_state)
		{
			case TR_INVALID:
				if (success)
				{
					if (m_leadingThresh==0)
					{
						m_state = TR_VALID;
						events |= TR_EVENT_VALIDATED;
						m_events |= TR_EVENT_VALIDATED;
					}
					else
					{
						setTimerMs(&m_timer);
						m_state = TR_LEADING;
					}
				}
				break;
			
			case TR_LEADING:
				if (success)
				{
					if (getTimerMs(m_timer)>=m_leadingThresh)
					{
						m_state = TR_VALID;
						events |= TR_EVENT_VALIDATED;
						m_events |= TR_EVENT_VALIDATED;
					}
				}
				else
				{
					m_state = TR_INVALID;
					events |= TR_EVENT_INVALIDATED;
					m_events |= TR_EVENT_INVALIDATED;
				}
				break;
			
			case TR_VALID:
				if (!success)
				{
					if (m_trailingThresh==0)
					{
						m_state = TR_INVALID;
						events |= TR_EVENT_INVALIDATED;
						m_events |= TR_EVENT_INVALIDATED;
					}
					else
					{
						setTimerMs(&m_timer);
						m_state = TR_TRAILING;
					}
				}
				if (m_age<TR_MAXAGE)
					m_age++;				
				break;
			
			case TR_TRAILING:
				if (success)
				{
					m_state = TR_VALID;
					events |= TR_EVENT_VALIDATED;
					m_events |= TR_EVENT_VALIDATED;
				}
				else
				{
					if (getTimerMs(m_timer)>=m_trailingThresh)
					{
						m_state = TR_INVALID;
						events |= TR_EVENT_INVALIDATED;
						m_events |= TR_EVENT_INVALIDATED;
					}
				}
				if (m_age<TR_MAXAGE)
					m_age++;
				break;
			
			default:
				m_state = TR_INVALID;
				events |= TR_EVENT_INVALIDATED;
				m_events |= TR_EVENT_INVALIDATED;
		}
		return events;
	}
	
	Object *get()
	{
		if (m_state==TR_INVALID || m_state==TR_LEADING)
			return NULL;
		else
			return &m_object;
	}
	
	TrackerState m_state;
	uint8_t m_index;
	uint8_t m_events;
	uint8_t m_eventsShadow;
	uint16_t m_timer;
	uint16_t m_leadingThresh;
	uint16_t m_trailingThresh;
	uint8_t m_age;
	uint32_t m_minVal;
	Object *m_minObject;
	Object m_object;
};
///tracker.h eof

///pixytypes.h sof
#define CL_NUM_SIGNATURES            7

#define RENDER_FLAG_FLUSH            0x01	// add to stack, render immediately
#define RENDER_FLAG_BLEND            0x02	// blend with a previous images in image stack
#define RENDER_FLAG_START            0x04	// indicate start of frame of data

#define PRM_FLAG_INTERNAL            0x00000001
#define PRM_FLAG_ADVANCED            0x00000002
#define PRM_FLAG_HEX_FORMAT          0x00000010
#define PRM_FLAG_SIGNED              0x00000080

// render-specific flags
#define PRM_FLAG_SLIDER              0x00000100
#define PRM_FLAG_CHECKBOX            0x00000200

// events
#define EVT_PARAM_CHANGE             1
#define EVT_RENDER_FLUSH             2
#define EVT_PROG_CHANGE              3

// text message flags
#define TM_FLAG_PRIORITY_NORMAL      0
#define TM_FLAG_PRIORITY_HIGH        1

#define PIXY_TYPE_REPONSE_ERROR              0x03
#define PIXY_TYPE_RESPONSE_BLOCKS            0x21
#define PIXY_TYPE_REQUEST_BLOCKS             0x20
#define PIXY_TYPE_REQUEST_LINES              0x30
#define PIXY_TYPE_RESPONSE_LINES             0x31

#define TM_IMPORTANT               1

struct ColorSignature
{
	ColorSignature()
	{
		m_uMin = m_uMax = m_uMean = m_vMin = m_vMax = m_vMean = m_type = 0;
	}	

    int32_t m_uMin;
    int32_t m_uMax;
    int32_t m_uMean;
    int32_t m_vMin;
    int32_t m_vMax;
    int32_t m_vMean;
	uint32_t m_rgb;
	uint32_t m_type;
};


struct Point16
{
    Point16()
    {
        m_x = m_y = 0;
    }

    Point16(int16_t x, int16_t y)
    {
        m_x = x;
        m_y = y;
    }

	uint32_t dist2(const Point16 &p) const
	{
		int32_t diffx, diffy;
		
		diffx = p.m_x - m_x;
		diffy = p.m_y - m_y;
		
		return diffx*diffx + diffy*diffy;
	}
	
    int16_t m_x;
    int16_t m_y;
};

struct Point32
{
    Point32()
    {
        m_x = m_y = 0;
    }

    Point32(int32_t x, int32_t y)
    {
        m_x = x;
        m_y = y;
    }

    int32_t m_x;
    int32_t m_y;
};

struct Frame8
{
    Frame8()
    {
        m_pixels = (uint8_t *)NULL;
        m_width = m_height = 0;
    }

    Frame8(uint8_t *pixels, uint16_t width, uint16_t height)
    {
        m_pixels = pixels;
        m_width = width;
        m_height = height;
    }

    uint8_t *m_pixels;
    int16_t m_width;
    int16_t m_height;
};

struct RectA
{
    RectA()
    {
        m_xOffset = m_yOffset = m_width = m_height = 0;
    }

    RectA(uint16_t xOffset, uint16_t yOffset, uint16_t width, uint16_t height)
    {
        m_xOffset = xOffset;
        m_yOffset = yOffset;
        m_width = width;
        m_height = height;
    }

    uint16_t m_xOffset;
    uint16_t m_yOffset;
    uint16_t m_width;
    uint16_t m_height;
};

struct RectB
{
    RectB()
    {
        m_left = m_right = m_top = m_bottom = 0;
    }

    RectB(uint16_t left, uint16_t right, uint16_t top, uint16_t bottom)
    {
        m_left = left;
        m_right = right;
        m_top = top;
        m_bottom = bottom;
    }

    uint16_t m_left;
    uint16_t m_right;
    uint16_t m_top;
    uint16_t m_bottom;
};



struct BlobA2
{
    BlobA2()
    {
        m_model = m_left = m_right = m_top = m_bottom = 0;
    }

    BlobA2(uint16_t model, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom)
    {
        m_model = model;
        m_left = left;
        m_right = right;
        m_top = top;
        m_bottom = bottom;
    }

    uint16_t m_model;
    uint16_t m_left;
    uint16_t m_right;
    uint16_t m_top;
    uint16_t m_bottom;
};

struct BlobB2
{
    BlobB2()
    {
        m_model = m_left = m_right = m_top = m_bottom = m_angle = 0;
    }

    BlobB2(uint16_t model, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom, int16_t angle)
    {
        m_model = model;
        m_left = left;
        m_right = right;
        m_top = top;
        m_bottom = bottom;
        m_angle = angle;
    }

    uint16_t m_model;
    uint16_t m_left;
    uint16_t m_right;
    uint16_t m_top;
    uint16_t m_bottom;
    int16_t m_angle;
};

struct BlobC
{
    BlobC()
    {
        m_model = m_x = m_y = m_width = m_height = m_angle = 0;
    }

    BlobC(uint16_t model, uint16_t x, uint16_t y, uint16_t width, uint16_t height, int16_t angle)
    {
        m_model = model;
        m_x = x;
        m_y = y;
        m_width = width;
        m_height = height;
        m_angle = angle;
		m_index = m_age = 0;
    }
	
    uint16_t m_model;
    uint16_t m_x;
    uint16_t m_y;
    uint16_t m_width;
    uint16_t m_height;
    int16_t m_angle;
	uint8_t m_index;
	uint8_t m_age;
};


struct HuePixel
{
    HuePixel()
    {
        m_u = m_v = 0;
    }

    HuePixel(int8_t u, int8_t v)
    {
        m_u = u;
        m_v = v;
    }

    int8_t m_u;
    int8_t m_v;
};

struct Fpoint
{
    Fpoint()
    {
        m_x = m_y = 0.0;
    }

    Fpoint(float x, float y)
    {
        m_x = x;
        m_y = y;
    }

    float m_x;
    float m_y;
};

struct UVPixel
{
    UVPixel()
    {
        m_u = m_v = 0;
    }

    UVPixel(int32_t u, int32_t v)
    {
        m_u = u;
        m_v = v;
    }

    int32_t m_u;
    int32_t m_v;
};

struct RGBPixel
{
    RGBPixel()
    {
        m_r = m_g = m_b = 0;
    }

    RGBPixel(uint8_t r, uint8_t g, uint8_t b)
    {
        m_r = r;
        m_g = g;
        m_b = b;
    }

    uint8_t m_r;
	uint8_t m_g;
	uint8_t m_b;
};


struct Line
{
    Line()
    {
        m_slope = m_yi = 0.0;
    }
    Line(float slope, float yi)
    {
        m_slope = slope;
        m_yi = yi;
    }

    float m_slope;
    float m_yi;
};

typedef long long longlong;
/// pixytypes.h eof

/// line.h sof
#define LINE_EDGE_DIST_DEFAULT            4
#define LINE_EDGE_THRESH_DEFAULT          35
#define LINE_EXTRACTION_DIST_DEFAULT      13
#define LINE_MAX_MERGE_DIST               6
#define LINE_MIN_LENGTH                   10
#define LINE_MIN_WIDTH                    0
#define LINE_MAX_WIDTH                    100
#define LINE_LINES_PER_COL_PITCH          (LINE_LINES_PER_COL+1) // +1 because of length byte at the beginning
#define LINE_VSIZE			              (CAM_RES3_WIDTH/4)
#define LINE_MAX_COMPARE                  5000

#define LINE_NODE_FLAG_V                  EQ_NODE_FLAG_V

#define LINE_HTHRESH_RATIO	              3/5

#define LINE_GRID_WIDTH_REDUCTION         3
#define LINE_GRID_HEIGHT_REDUCTION        1
#define LINE_GRID_WIDTH                   (CAM_RES3_WIDTH>>LINE_GRID_WIDTH_REDUCTION)
#define LINE_GRID_HEIGHT                  (CAM_RES3_HEIGHT>>LINE_GRID_HEIGHT_REDUCTION)

#define LINE_GRID_INDEX(x, y)             (LINE_GRID_WIDTH*(y)+(x))
#define LINE_GRID_INDEX_P(p)              LINE_GRID_INDEX(p.m_x, p.m_y)
#define LINE_GRID(x, y)                   g_lineGrid[LINE_GRID_INDEX(x, y)]
#define LINE_GRID_P(p)                    LINE_GRID(p.m_x, p.m_y)
#define LINE_GRID_LINE(x, y)              (LINE_GRID(x, y)&LINE_NODE_LINE_MASK)
#define LINE_GRID_LINE_P(p)               (LINE_GRID_P(p)&LINE_NODE_LINE_MASK)


#define LINE_NODE_LINE_MASK               0x03ff
#define LINE_NODE_FLAG_HLINE              EQ_NODE_FLAG_HLINE
#define LINE_NODE_FLAG_VLINE              EQ_NODE_FLAG_VLINE
#define LINE_NODE_FLAG_1                  (LINE_NODE_FLAG_HLINE | LINE_NODE_FLAG_VLINE)
#define LINE_NODE_FLAG_NULL               0x8000
#define LINE_MAX_SEGMENTS                 0x100
#define LINE_MAX_LINES                    0x80
#define LINE_MAX_SEGMENT_POINTS           32
#define LINE_MAX_INTERSECTION_LINES       8 // needs to be an even number
#define LINE_MAX_FRAME_INTERSECTION_LINES 6 // needs to be an even numbe

#define LINE_DEBUG_BENCHMARK              1  // not bit
#define LINE_DEBUG_LAYERS                 2  // bit
#define LINE_DEBUG_GRAPH_CHECK            4  // bit
#define LINE_DEBUG_TRACKING               8 // bit

#define	LINE_RM_MINIMAL                   0
#define	LINE_RM_MINIMAL_STR               "Primary features, no backgound"  
#define	LINE_RM_PRIMARY_FEATURES          1
#define	LINE_RM_PRIMARY_FEATURES_STR      "Primary features"
#define	LINE_RM_ALL_FEATURES              2
#define	LINE_RM_ALL_FEATURES_STR          "All features" 


#define LINE_MMC_BITS                     4
#define LINE_MMC_MIN_EDGES                (4+LINE_MMC_BITS)
#define LINE_MMC_MAX_EDGES                (2+(LINE_MMC_BITS*2))

#define LINE_MMC_CANDIDATE_BARCODES       32
#define LINE_MMC_VOTED_BARCODES           8
#define LINE_MMC_VTSIZE                   8  // voting table size
#define LINE_MMC_VBOUNDARY                0.25
#define LINE_MMC_HBOUNDARY                0.1

#define LINE_MIN_ACQUISITION_TAN_ANGLE    500
#define LINE_MIN_ACQUISITION_LENGTH2      15*15

// define 2 bits to describe which of the two points we are referring to on a line -- is it the upper-most point?
// or the right-most point?
#define LINE_HT_UP                        0x01
#define LINE_HT_DOWN                      0x02
#define LINE_HT_LEFT                      0x04
#define LINE_HT_RIGHT                     0x08

#define LINE_FILTERING_MULTIPLIER         16

#define LINE_LINE_FILTERING				  1
#define LINE_INTERSECTION_FILTERING       1
#define LINE_BARCODE_FILTERING            1

#define LINE_MIN_INTERSECTION_DETECT      (LINE_GRID_HEIGHT/4) 

#define LINE_FR_VECTOR_LINES                 0x01
#define LINE_FR_INTERSECTION                 0x02
#define LINE_FR_BARCODE                      0x04

#define LINE_FR_FLAG_INTERSECTION            0x04

#define LINE_MODEMAP_TURN_DELAYED            0x01
#define LINE_MODEMAP_MANUAL_SELECT_VECTOR    0x02
#define LINE_MODEMAP_WHITE_LINE              0x80

enum LineState
{
	LINE_STATE_ACQUIRING, 
	LINE_STATE_TRACKING
};

struct BarCodeCluster
{
    BarCodeCluster()
    {
        m_n = 0;
    }

    void addCode(uint8_t index)
    {
        if (m_n>=LINE_MMC_CANDIDATE_BARCODES)
            return;
        m_indexes[m_n] = index;
        m_n++;
    }

    void updateWidth(uint16_t width)
    {
        m_width = (m_width*(m_n-1) + width)/m_n; // recursive averager
    }

    uint8_t m_indexes[LINE_MMC_CANDIDATE_BARCODES];
    uint8_t m_n;
    Point16 m_p0;
    Point16 m_p1;
    uint16_t m_width;
};

struct BarCode
{
    BarCode()
    {
        m_n = 0;
    }

    Point16 m_p0;
    uint16_t m_width;
    int16_t m_val;
    uint16_t m_edges[LINE_MMC_MAX_EDGES];
    uint8_t m_n;
};


struct DecodedBarCode
{
    RectA m_outline;
    int16_t m_val;
	Tracker<DecodedBarCode> *m_tracker;
};


typedef uint16_t LineGridNode;


extern LineGridNode *g_lineGrid;

struct Nadir
{
	Nadir()
	{
		m_n = 0;
		m_dist = 0;
	}
	
	void merge(const Nadir &n)
	{
		uint16_t xavg, yavg;
		uint8_t i, j, li, lj;
		
		// look through list, make sure there are no duplicate lines
		for (i=0; i<n.m_n; i++)
		{
			if (m_n>=LINE_MAX_INTERSECTION_LINES)
				return;
			li = LINE_GRID_LINE_P(n.m_points[i]);
			for (j=0; j<m_n; j++)
			{
				lj = LINE_GRID_LINE_P(m_points[j]);
				if (li==lj)
					break;
			}
			if (j==m_n) // we reached end of list
				m_points[m_n++] = n.m_points[i];
		}
		// average all points
		for (i=xavg=yavg=0; i<m_n; i++)
		{
			xavg += m_points[i].m_x;
			yavg += m_points[i].m_y;
		}
		
		m_pavg.m_x = (xavg+(m_n>>1))/m_n;
		m_pavg.m_y = (yavg+(m_n>>1))/m_n;
		
	}

	Point m_points[LINE_MAX_INTERSECTION_LINES];
	Point m_pavg;
	uint8_t m_n;
	uint16_t m_dist;
};


struct Intersection;

struct Line2
{
	Line2()
	{
		m_i0 = m_i1 = NULL;
		m_tracker = NULL;
		m_index = 0;
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
	uint8_t m_index;
	SimpleListNode<Intersection> *m_i0;
	SimpleListNode<Intersection> *m_i1;
	Tracker<Line2> *m_tracker;
};


struct Intersection
{
	Intersection()
	{
	}

	Intersection(const Point &p)
	{
		m_p = p;
	}
	
	bool addLine(SimpleListNode<Line2> *linen, SimpleListNode<Intersection> *intern, uint8_t pi)
	{
		uint8_t i;
		
		if (linen==NULL || intern==NULL || m_n>=LINE_MAX_INTERSECTION_LINES)
			return false;
		for (i=0; i<m_n; i++)
		{
			if (m_lines[i]==linen)
				return false;
		}
		m_lines[m_n++] = linen;
		if (pi==0)
			linen->m_object.m_i0 = intern;
		else
			linen->m_object.m_i1 = intern;
		return true;
	}
	
	Point m_p; 
	uint8_t m_n;
	SimpleListNode<Line2> *m_lines[LINE_MAX_INTERSECTION_LINES];
};

struct FrameIntersectionLine
{
	uint8_t m_index;
	uint8_t m_reserved;
	int16_t m_angle;
};

struct FrameIntersection
{
	uint8_t m_x;
	uint8_t m_y;
	uint8_t m_n;
	uint8_t m_reserved;
	FrameIntersectionLine m_lines[LINE_MAX_FRAME_INTERSECTION_LINES];
};

struct FrameLine
{
	uint8_t m_x0;
	uint8_t m_y0;
	uint8_t m_x1;
	uint8_t m_y1;
	uint8_t m_index;
	uint8_t m_flags;
};

struct FrameCode
{
	uint8_t m_x;
	uint8_t m_y;
	uint8_t m_flags;
	uint8_t m_code;
};
/// line.h eof

/// line.cpp sof

#define ABS(x)      ((x)<0 ? -(x) : (x))
#define SIGN(x)     ((x)>=0 ? 1 : -1)

#define LINE_BUFSIZE    CAM_RES3_WIDTH/2

Equeue *g_equeue;

static uint16_t *g_lineBuf;

static uint16_t g_minLineWidth=LINE_MIN_WIDTH;
static uint16_t g_maxLineWidth=LINE_MAX_WIDTH;
static uint32_t g_minLineLength2=LINE_MIN_LENGTH*LINE_MIN_LENGTH; // squared
static uint32_t g_minLineLength=LINE_MIN_LENGTH; // not squared
static uint16_t g_maxMergeDist=LINE_MAX_MERGE_DIST; 

static uint16_t g_extractionDist=LINE_EXTRACTION_DIST_DEFAULT;

static EqueueFields g_savedEqueue;

///static ChirpProc g_getEdgesM0 = -1;
///static ChirpProc g_setEdgeParamsM0 = -1;

LineGridNode *g_lineGrid;
static uint8_t *g_lineGridMem;
static LineSeg *g_lineSegs;
static uint8_t *g_lineSegsMem;
static uint8_t g_pointsPerSeg;
static LineSegIndex g_lineSegIndex;
static float g_maxError;
static uint8_t g_lineIndex;
static SimpleListNode<Line2> **g_lines;
static uint32_t g_maxSegTanAngle;
static uint32_t g_maxEquivTanAngle;
static uint32_t g_maxTrackingTanAngle;

static SimpleList<Line2> g_linesList;
static SimpleList<Point> g_nodesList;
static SimpleList<Nadir> g_nadirsList;
static SimpleList<Intersection> g_intersectionsList;

static uint8_t g_barcodeIndex;
static BarCode **g_candidateBarcodes;
static DecodedBarCode *g_votedBarcodes;
static uint8_t *g_votedBarcodesMem;
static uint8_t g_votedBarcodeIndex;
static uint16_t g_maxCodeDist;
static uint16_t g_minVotingThreshold;

static SimpleList<Tracker<Line2> > g_lineTrackersList;
static Tracker<FrameIntersection> g_primaryIntersection;


static uint8_t g_lineFiltering=LINE_LINE_FILTERING;
uint16_t myleading = g_lineFiltering*LINE_FILTERING_MULTIPLIER;
uint16_t mytrailing = (myleading+1)>>1;

static bool g_newIntersection;

static SimpleList<Tracker<DecodedBarCode> > g_barCodeTrackersList;
static uint8_t g_barCodeTrackerIndex;
static uint8_t g_lineTrackerIndex;
static uint8_t g_primaryLineIndex;
static uint8_t g_primaryPointMap;
static Point g_goalPoint;
static Point g_primaryPoint;
static bool g_primaryActive;
static uint32_t g_maxLineCompare=LINE_MAX_COMPARE;

static LineState g_lineState;

static uint8_t g_renderMode;
static uint8_t g_barcodeFiltering=LINE_BARCODE_FILTERING;

static bool g_frameFlag;
static bool g_primaryMutex;
static bool g_allMutex;

static int16_t g_nextTurnAngle;
static int16_t g_defaultTurnAngle=0;
static uint8_t g_delayedTurn=0;
static bool g_newTurnAngle;
static bool g_manualVectorSelect=0;
static uint8_t g_manualVectorSelecIndextActive;
static uint8_t g_manualVectorSelectIndex;
static bool g_reversePrimary;
/// line.cpp eof


///pixyvals.h sof
///moved to hpp
//#define SRAM1_LOC                smram1///0x10080000
//#define SRAM1_SIZE               0x12000
//#define SRAM4_LOC                smram4///0x2000c000
//#define SRAM4_SIZE               0x4000
uint8_t SRAM4_LOC[SRAM4_SIZE]; //emulating smram4
uint8_t SRAM1_LOC[SRAM1_SIZE];
///pixyvals.h eof
///smlink.h sof
#define SM_LOC                 (SRAM4_LOC+0x3c00)
#define SM_SIZE                (SRAM4_SIZE-0x3c00)
#define SM_BUFSIZE             (SM_SIZE-12)

// status
#define SM_STATUS_DATA_AVAIL   0x01

int linkSend(const uint8_t *data, uint32_t len, uint16_t timeoutMs);
int linkReceive(uint8_t *data, uint32_t len, uint16_t timeoutMs);

typedef struct
{
	volatile uint8_t stream;
	volatile uint8_t streamState;
	volatile uint16_t currentLine;
	volatile uint16_t frameTime;
	volatile uint16_t blankTime;
	volatile uint16_t sendStatus; // NOTE, M4 send and recv flags are backwards
	volatile uint16_t recvStatus;

	volatile uint8_t buf[SM_BUFSIZE];
}
SmMap;

#define SM_OBJECT       ((SmMap *)SM_LOC)
///smlink.h eof

/// equeue.h sof
#define EQ_LOC        SRAM4_LOC
#define EQ_SIZE       0x3c00

#define EQ_MEM_SIZE  ((EQ_SIZE-sizeof(struct EqueueFields)+sizeof(uint16_t))/sizeof(uint16_t))

#define EQ_HSCAN_LINE_START 0xff00
#define EQ_VSCAN_LINE_START 0xff01
#define EQ_ERROR            0xfffe
#define EQ_FRAME_END        0xffff
#define EQ_NEGATIVE         0x8000

#define EQ_STREAM_EDGES     0x0001
#define EQ_STREAM_LINES	    0x0002

#define EQ_NODE_SIZE        12

#define EQ_NODE_FLAG_HLINE  0x0400
#define EQ_NODE_FLAG_VLINE  0x0800


typedef int16_t LineSegIndex;

struct EqueueFields *g_Equeue = (struct EqueueFields *)EQ_LOC;

uint16_t eq_free(void)
{
    uint16_t len = g_Equeue->produced - g_Equeue->consumed;
	return EQ_MEM_SIZE-len;
} 

uint32_t eq_enqueue(uint16_t val)
{
    if (eq_free()>0)
    {
        g_Equeue->data[g_Equeue->writeIndex++] = val;
        g_Equeue->produced++;
		if (g_Equeue->writeIndex==EQ_MEM_SIZE)
			g_Equeue->writeIndex = 0;
        return 1;
    }
    return 0;
}


#define ENQUEUE_START() \
	uint16_t *data = g_Equeue->data; \
	uint16_t writeIndex = g_Equeue->writeIndex; \
	uint16_t produced = 0 

#define ENQUEUE(val) \
	data[writeIndex++] = val; \
	produced++; \
	if (writeIndex==EQ_MEM_SIZE) \
		writeIndex = 0 

#define ENQUEUE_END() \
	g_Equeue->writeIndex = writeIndex; \
	g_Equeue->produced += produced 
/// equeue.c eof

/// frame_m0.c sof
uint16_t g_dist = LINE_EDGE_DIST_DEFAULT;
uint16_t g_thresh = LINE_EDGE_THRESH_DEFAULT;
uint16_t g_hThresh = LINE_EDGE_THRESH_DEFAULT*LINE_HTHRESH_RATIO;
static uint8_t g_whiteLine=0;
static uint8_t g_go=0;
static uint8_t g_repeat=0;


int hScan(uint8_t *memy)///, uint8_t *memc)
{
	int16_t i;
	int16_t end, diff;
	ENQUEUE_START();
	
	if (eq_free()<CAM_RES3_WIDTH/2+CAM_RES3_HEIGHT)
		return -1;
	ENQUEUE(EQ_HSCAN_LINE_START);

	i = -1;
	end = CAM_RES3_WIDTH - g_dist;

	// state 0, looking for either edge
loop0:
	i++;
	if (i>=end)
		goto loopex;
	diff = memy[i+g_dist]-memy[i];
	if (-g_thresh>=diff)
		goto edge0;
	if (diff>=g_thresh)
		goto edge1;
	goto loop0;

	// found neg edge
edge0:
	ENQUEUE(i | EQ_NEGATIVE);
	i+=2;

	// state 1, looking for end of edge or pos edge
loop1:
	i++;
	if (i>=end)
		goto loopex;
	diff = memy[i+g_dist]-memy[i];
	if (-g_hThresh<diff)
		goto loop0;
	if (diff>=g_thresh)
		goto edge1;
	goto loop1;

	// found pos edge
edge1:
	ENQUEUE(i);
	i+=2;
	
	// state 2, looking for end of edge or neg edge
loop2:
	i++;
	if (i>=end)
		goto loopex;
	diff = memy[i+g_dist]-memy[i];
	if (diff<g_hThresh)
		goto loop0;
	if (-g_thresh>=diff)	
		goto edge0;
	goto loop2;

loopex:
	ENQUEUE_END();
	return 0;
	
}


int vScan(uint8_t *memy)///, uint8_t *memc)
{
	int16_t i;
	int16_t diff;
	uint8_t *line0;
	ENQUEUE_START();
	
	if (eq_free()<CAM_RES3_WIDTH/2+CAM_RES3_HEIGHT)
		return -1;
	ENQUEUE(EQ_VSCAN_LINE_START);

	i = -4;
	line0 = memy - ((g_dist+5)>>2)*CAM_RES3_WIDTH;

loop:
	i+=4;
	if (i>=CAM_RES3_WIDTH)
		goto loopex;
	diff = memy[i]-line0[i];
	if (-g_thresh>=diff)
		goto edge0;
	if (diff>=g_thresh)
		goto edge1;

	goto loop;

edge0:
	ENQUEUE(i | EQ_NEGATIVE);
	goto loop;

edge1:
  ENQUEUE(i);
	goto loop;

loopex:
	ENQUEUE_END();
	return 0;
}
//__attribute__((noinline))
void	lineM0R3(uint8_t const *d, uint16_t width, uint8_t *memy, uint8_t *memc, uint16_t l)
{
	const uint8_t ri=0, gi=1, bi=2, cc=3;
	// r0: d - gpio
  uint8_t const *r0 = &d[l * CAM_RES3_WIDTH * cc];
	// r1: width
  uint8_t *r1 = memy + width;
	// r2: memy
  uint8_t *r2 = memy;
	// r3: memc
  uint8_t *r3 = memc;
	// r4: scratch
  // r5: scratch
	uint8_t r4, r5;
	
	///_ASM(LDRB 	r4, [r0]) // blueA
  r4 = r0[(l&1)?gi:bi]; r0=&r0[cc];
	//
	///_ASM(ADDS 	r1, r2) // calculate end of line memory ///already done
	
///_ASM_LABEL(loop_bg)
loop_bg:
	///_ASM(LDRB		r5, [r0]) // greenA
	r5 = r0[(l&1)?ri:gi]; r0=&r0[cc];
	///_ASM(STRB		r4, [r3, #0])
	r3[0] = r4;
	///_ASM(STRB		r5, [r2, #0])
	r2[0] = r5;
	///_ASM(LDRB		r4, [r0]) // blueB
	r4 = r0[(l&1)?gi:bi]; r0=&r0[cc];
	///_ASM(STRB		r4, [r3, #2])
  r3[2] = r4;
	///_ASM(CMP 		r2, r1)
	///_ASM(BGE		out_bg)
  if(r2>=r1) goto out_bg;
	///_ASM(LDRB		r4, [r0]) // greenB
	r4 = r0[(l&1)?ri:gi]; r0=&r0[cc];
	///_ASM(STRB		r4, [r2, #2])
  r2[2] = r4;
	///_ASM(ADDS		r2, #4)
  r2+=4;
	///_ASM(ADDS		r3, #4)
  r3+=4;
	///_ASM(LDRB		r4, [r0]) // blueA
	r4 = r0[(l&1)?gi:bi]; r0=&r0[cc];
	goto loop_bg;
  ///_ASM_LABEL(out_bg)
  out_bg:
  return;
}
int32_t grabM0R3(uint8_t *memy, uint8_t const *d)
{
	///int32_t minTime;	
	uint32_t line;
	///uint32_t timer;
	///int32_t time;
	uint8_t *memc = memy+CAM_RES3_WIDTH*CAM_RES3_HEIGHT+16; /// stores just the most recent 2 interlaced consecutive short lines (2*318)
	///uint32_t maxScanTime = 3*g_hblank+2*g_hactive-MAX_SYNC_TIME;
	///uint32_t fourLineTime = 4*g_hblank+4*g_hactive-MAX_SYNC_TIME;
	uint8_t scan;
	///minTime = 100;
	///skipLines(1);
	for (line=0, scan=1; line<CAM_RES3_HEIGHT; line++, memy+=CAM_RES3_WIDTH) //input needs to be: 636 x 104*4; output is 2*(636/2)*104
	{
		// CAM_HSYNC is negated here
		lineM0R3(d, CAM_RES3_WIDTH, memy+1, memc, 4*line);
		///while(CAM_HSYNC());
		lineM0R3(d, CAM_RES3_WIDTH, memc+1, memy, 4*line+1); 
		///while(CAM_HSYNC());
		
		if (scan)
		{
      ///*****scan is done while 2 lines are being skipped
			///setTimer(&timer);
			///if (hScan(memy, memc)<0)
      if (hScan(memy)<0)
			{
				eq_enqueue(EQ_HSCAN_LINE_START);
				scan = 0;
        return -1;
			}
			///if (line>=(g_dist+5)>>2 && vScan(memy, memc)<0)
      if (line>=((uint32_t)(g_dist+5)>>2) && vScan(memy)<0)
			{
				eq_enqueue(EQ_VSCAN_LINE_START);
				scan = 0;
        return -1;
			}
      /* ///timing is not simulated
			time = maxScanTime - getTimer(timer);
			
			if (time<0)
			{
				time += fourLineTime;
				setTimer(&timer);
				line++, memy+=CAM_RES3_WIDTH;
				eq_enqueue(EQ_HSCAN_LINE_START);
				if (line>=(g_dist+5)>>2)
					eq_enqueue(EQ_VSCAN_LINE_START);
				time -= getTimer(timer);
			}
			if (minTime>time)
				minTime = time;
			if (time>0)
				delayus(time);
      */
		}
		else
		{
			eq_enqueue(EQ_HSCAN_LINE_START);
			if (line>=((uint32_t)(g_dist+5)>>2))
				eq_enqueue(EQ_VSCAN_LINE_START);
			SM_OBJECT->currentLine = line;
			/*
			while(!CAM_HSYNC());
			while(CAM_HSYNC());
			while(!CAM_HSYNC());
			while(CAM_HSYNC());*/
		}

		// update line count
		SM_OBJECT->currentLine = line;	
	}
	//printf("sz %d\n", ((line-1)*4+2)*3*CAM_RES3_WIDTH);
	eq_enqueue(EQ_FRAME_END);

	return 0;
}
/// fame_m0.c eof

/// equeue.cpp sof
Equeue::Equeue()
{
    m_fields = (EqueueFields *)EQ_LOC;
    reset();
}

Equeue::~Equeue()
{
}

void Equeue::reset()
{
    memset((void *)m_fields, 0, sizeof(EqueueFields));
}

uint32_t Equeue::dequeue(uint16_t *val)
{
    uint16_t len = m_fields->produced - m_fields->consumed;
    if (len)
    {
        *val = m_fields->data[m_fields->readIndex++];
        m_fields->consumed++;
        if (m_fields->readIndex==EQ_MEM_SIZE)
            m_fields->readIndex = 0;
        return 1;
    }
    return 0;
}

// first word is always a code hscan or vscan
// read until next code
// if code is an error, eat it and return error
// if we run out of data, we return 0
uint32_t Equeue::readLine(uint16_t *mem, uint32_t size, bool *eof, bool *error)
{
	uint16_t len = m_fields->produced - m_fields->consumed;
	uint16_t i, j;
	uint8_t codes;
	
	for (i=0, j=m_fields->readIndex, codes=0, *eof=false, *error=false; true; i++)
	{
		if (i>=len || i>=size)
			return 0;
		
		mem[i] = m_fields->data[j];
		
		if (m_fields->data[j]>=EQ_HSCAN_LINE_START)
		{
			if (m_fields->data[j]==EQ_ERROR)
			{
				i++; // return and eat error code
				*error = true;
				break;
			}
			else if (m_fields->data[j]==EQ_FRAME_END)
			{
				i++; // return and eat eof code
				*eof = true;
				break;
			}
			codes++;
			if (codes>=2)
				// don't return code
				break;
		}
		j++;
		if (j==EQ_MEM_SIZE)
			j = 0;
	}
		
	// flush what we read
	m_fields->consumed += i;
	m_fields->readIndex += i;
	if (m_fields->readIndex>=EQ_MEM_SIZE)	
		m_fields->readIndex -= EQ_MEM_SIZE;

	return i;
}

void Equeue::flush()
{
    uint16_t len = m_fields->produced - m_fields->consumed;

    m_fields->consumed += len;
    m_fields->readIndex += len;
    if (m_fields->readIndex>=EQ_MEM_SIZE)
        m_fields->readIndex -= EQ_MEM_SIZE;
}
/// equeue.cpp eof

/// line.cpp sof
uint32_t tanDiffAbs1000(const Point &p00, const Point &p01, const Point &p10, const Point &p11, bool min=false)
{
	// find tangent of angle difference between the two lines
	int16_t xdiff0, ydiff0;
	int16_t xdiff1, ydiff1;
	int32_t x, y, res;

	xdiff0 = p01.m_x - p00.m_x;
	ydiff0 = p01.m_y - p00.m_y;
	xdiff1 = p11.m_x - p10.m_x;
	ydiff1 = p11.m_y - p10.m_y;

	x = xdiff0*xdiff1 + ydiff0*ydiff1;
	y = ydiff0*xdiff1 - ydiff1*xdiff0;

	// min==true says that we don't care how the lines stack up, just return the abs of the minimum.
	// min==false says that we care of the angles open beyond 90 degrees, in which case return a big value.
	if ((min&&x==0) || (!min&&x<=0))
		return 1000000;

	res = y*1000/x;
	res = ABS(res);
	return res;
}

uint32_t tanAbs1000(const Point &p0, const Point &p1)
{
	// find tangent of angle difference between the two lines
	int16_t xdiff, ydiff;
	int32_t res;

	xdiff = p1.m_x - p0.m_x;
	ydiff = p1.m_y - p0.m_y;

	if (xdiff==0)
		return 1000000;

	res = ydiff*1000/xdiff;
	res = ABS(res);
	return res;
}
bool xdirection(const Point &p0, const Point &p1)
{
    int16_t xdiff, ydiff;

    xdiff = p1.m_x - p0.m_x;
    ydiff = p1.m_y - p0.m_y;

    return abs(xdiff)>abs(ydiff);
}
bool validIntersection(SimpleListNode<Intersection> *intern)
{
	SimpleListNode<Intersection> *i;
	
	if (intern==NULL)
		return true; // valid

	for (i=g_intersectionsList.m_first; i!=NULL; i=i->m_next)
	{
		if (i==intern)
			return true;
	}
	return false;
}

bool validLine(SimpleListNode<Line2> *linen)
{
	SimpleListNode<Line2> *i;
	
	if (linen==NULL)
		return true; // valid

	for (i=g_linesList.m_first; i!=NULL; i=i->m_next)
	{
		if (i==linen)
			return true;
	}
	return false;
}

#define CHECK(cond, index)   if(!(cond) && suppress0!=index && suppress1!=index) {check=index; goto end;} 

bool checkGraph0(int val, uint8_t suppress0=0, uint8_t suppress1=0, SimpleListNode<Intersection> *intern=NULL)
{
	printf("call\n");
	SimpleListNode<Line2> *i;
	SimpleListNode<Intersection> *j;
	uint8_t k, n, h, check=0;	
	
	//if (!(g_debug&LINE_DEBUG_GRAPH_CHECK))
	//	return true;
	
	if (intern)
		goto intersection;
	
	// go through line list, make sure all intersection pointers are valid 
	// by finding them in the list, make sure the line is in the intersection's list
	for (i=g_linesList.m_first; i!=NULL; i=i->m_next)
	{
		CHECK(validIntersection(i->m_object.m_i0), 1);
		CHECK(validIntersection(i->m_object.m_i1), 2);
		if (i->m_object.m_i0)
		{
			for (k=n=0; k<i->m_object.m_i0->m_object.m_n; k++)
			{
				if (i->m_object.m_i0->m_object.m_lines[k]==i)
					n++;
			}
			CHECK(n==1, 3);
		}
		if (i->m_object.m_i1)
		{
			for (k=n=0; k<i->m_object.m_i1->m_object.m_n; k++)
			{
				if (i->m_object.m_i1->m_object.m_lines[k]==i)
					n++;
			}
			CHECK(n==1, 4);
		}		
	}
	
	intersection:
	// go through intersection list, make sure each intersection's lines are valid
	// make sure each line points back to the intersection
	for (j=g_intersectionsList.m_first; j!=NULL; j=j->m_next)
	{
		if (intern)
			j = intern;
		for (k=n=0; k<j->m_object.m_n; k++)
		{
			CHECK(validLine(j->m_object.m_lines[k]), 5);
			CHECK(j->m_object.m_lines[k]->m_object.m_i0 != j->m_object.m_lines[k]->m_object.m_i1, 6);
			CHECK(j->m_object.m_lines[k]->m_object.m_i0==j || j->m_object.m_lines[k]->m_object.m_i1==j, 7);
			if (j->m_object.m_lines[k]->m_object.m_i0==j)
				CHECK(j->m_object.m_p.equals(j->m_object.m_lines[k]->m_object.m_p0), 8);
			if (j->m_object.m_lines[k]->m_object.m_i1==j)
				CHECK(j->m_object.m_p.equals(j->m_object.m_lines[k]->m_object.m_p1), 9);			
			for (h=k+1; h<j->m_object.m_n; h++)
			{
				if (j->m_object.m_lines[k]==j->m_object.m_lines[h])
					n++;
			}
			CHECK(n==0, 10);
		}
		if (intern)
			break;
	}
	
	end:
	if (check!=0)
	{
		cprintf(0, "fail %d %d\n", check, val);
		return false;
	}
		
	return true;
}
#if 0
#define checkGraph(...) checkGraph0(__VA_ARGS__)
#else
#define checkGraph(...) ;
#endif
uint8_t removeLine(SimpleListNode<Line2> *linen, SimpleListNode<Intersection> *intern)
{
	uint8_t i, j, m;
	
	// remove line from intersection
	for (i=m=0; intern && i<intern->m_object.m_n; i++)
	{
		if (intern->m_object.m_lines[i]==linen)
		{
			intern->m_object.m_n--;
			// shift lines down
			for (j=i; j<intern->m_object.m_n; j++)
				intern->m_object.m_lines[j] = intern->m_object.m_lines[j+1];
			i--;
			m++;
		}
	}
	return m;
}
void removeLine(SimpleListNode<Line2> *linen)
{	
	removeLine(linen, linen->m_object.m_i0);
	removeLine(linen, linen->m_object.m_i1);
	
#if 0
	// remove line from g_lines
	for (i=0; i<g_lineIndex; i++)
	{
		if (g_lines[i]==linen)
		{
			g_lines[i] = NULL;
			break;
		}
	}
#endif
	
	// remove line from g_linesList
	g_linesList.remove(linen);
	
}

uint16_t removeShortLinesIntersections()
{
	uint8_t i, si;
	SimpleListNode<Intersection> *j;
	uint16_t n, m, length, shortest;
	
	// look through intersections removing short lines
	// we remove the shortest lines first because we want to preserve the next-longest line for when we get to lines==2
	for (j=g_intersectionsList.m_first, m=0; j!=NULL; j=j->m_next)
	{
		while(1)
		{
			checkGraph(__LINE__);
			n = 0;
			// If there are 2 lines in the intersection, no reason to trim (we'll merge them)
			//if (j->m_object.m_n<=2)
			//	break;
			
			// find shortest line in intersection
			for (i=0, shortest=0xffff, si=0; i<j->m_object.m_n; i++)
			{
				length = j->m_object.m_lines[i]->m_object.length2();
				if (length<shortest)
				{
					si = i;
					shortest = length;
				}
			}
				
			// if it's shorter than g_minLineLength2, possibly remove it 
			if (shortest<g_minLineLength2)
			{
				SimpleListNode<Line2> *linen = j->m_object.m_lines[si];
				// one of the intersection pointers needs to be NULL, the other is our intersection
				// if they are both non-NULL, this line bridges 2 intersections, so we can't remove
				if (linen->m_object.m_i0==NULL || linen->m_object.m_i1==NULL)
				{
					removeLine(linen);
					n++;
					m++;
				}
			}
			checkGraph(__LINE__);
			if (n==0)
				break;
		}
	}
	return m;
}
bool equivalentLines(const Line2 &line0, const Line2 &line1)
{
	return ((line0.m_p0.equals(line1.m_p0) && line0.m_p1.equals(line1.m_p1)) ||
		(line0.m_p0.equals(line1.m_p1) && line0.m_p1.equals(line1.m_p0)));
}

bool minAngleLines(const Line2 &line0, const Line2 &line1, const Point &p, uint32_t minAngle)
{
	Point p01, p11;
	
	if (p.equals(line0.m_p0))
		p01 = line0.m_p1;
	else
		p01 = line0.m_p0;
	if (p.equals(line1.m_p0))
		p11 = line1.m_p1;
	else
		p11 = line1.m_p0;
	
	return (tanDiffAbs1000(p, p01, p, p11) <= minAngle);
}

uint16_t removeRedundantLinesIntersections()
{
	uint8_t i, j, n;
	SimpleListNode<Intersection> *k;

	for (k=g_intersectionsList.m_first, n=0; k!=NULL; k=k->m_next)
	{
		// perform n^2 search of lines within this intersection
		for (i=0; i<k->m_object.m_n; i++)
		{
			for (j=i+1; j<k->m_object.m_n; j++)
			{
				// Check for equivalent lines...
				if (equivalentLines(k->m_object.m_lines[i]->m_object, k->m_object.m_lines[j]->m_object))
				{
					removeLine(k->m_object.m_lines[j]);
					n++;
					j--; // list was shifted down, decrement so we don't skip an entry
				}
				// Check for lines with a small angle between them
				else if (minAngleLines(k->m_object.m_lines[i]->m_object, k->m_object.m_lines[j]->m_object, k->m_object.m_p, g_maxEquivTanAngle))
				{
					// lines i and j have a small angle between them.  Which do we delete?  Which line is longer?  What if the shorter one is connected to another intersection?
					// if i is longer than j
					if (k->m_object.m_lines[i]->m_object.length2() > k->m_object.m_lines[j]->m_object.length2())
					{
						// remove line j, since it's shorter, unless it's connected...
						if (k->m_object.m_lines[j]->m_object.m_i0==NULL || k->m_object.m_lines[j]->m_object.m_i1==NULL)
						{
							removeLine(k->m_object.m_lines[j]);
							n++;
							j--; // list was shifted down, decrement so we don't skip an entry
						}
						// otherwise remove line i, if it's not connected (we know that it's connected this intersection, so if it has at least 1 NULL intersection pointer...)
						else if (k->m_object.m_lines[i]->m_object.m_i0==NULL || k->m_object.m_lines[i]->m_object.m_i1==NULL)
						{
							removeLine(k->m_object.m_lines[i]);
							n++;
							i--; // list was shifted down, decrement so we don't skip an entry
							break; // then exit, since it affects our i-th index
						}
					}
					else // Line j is longer than i
					{
						// remove line i, since it's shorter, unless it's connected... 
						if (k->m_object.m_lines[i]->m_object.m_i0==NULL || k->m_object.m_lines[i]->m_object.m_i1==NULL)
						{
							removeLine(k->m_object.m_lines[i]);
							n++;
							i--; // list was shifted down, decrement so we don't skip an entry
							break; // then exit, since it affects our i-th index
						}
						// otherwise remove line j, if it's not connected
						else if (k->m_object.m_lines[j]->m_object.m_i0==NULL || k->m_object.m_lines[j]->m_object.m_i1==NULL)
						{
							removeLine(k->m_object.m_lines[j]);
							n++;
							j--; // list was shifted down, decrement so we don't skip an entry
						}
					}
				}
			}
		}
	}
	checkGraph(__LINE__);
	
	return n;
}
void replaceLine(SimpleListNode<Line2> *linen, SimpleListNode<Line2> *linenx)
{
	uint8_t i;
	SimpleListNode<Intersection> *j;

#if 0	
	for (i=0; i<g_lineIndex; i++)
	{
		if (g_lines[i]==linenx)
		{
			g_lines[i] = linen;
			break;
		}
	}
#endif
	
	// remove line from existing intersections
	for (j=g_intersectionsList.m_first; j!=NULL; j=j->m_next)
	{
		for (i=0; i<j->m_object.m_n; i++)
		{
			if (j->m_object.m_lines[i]==linenx)
			{
				j->m_object.m_lines[i] = linen;
//				break;
			}
		}
	}
	
	g_linesList.remove(linenx);
}

uint16_t simplifyIntersections()
{
	uint16_t n;
	SimpleListNode<Intersection> *j, *jnext;
	//#define DTLINE
	//#define DTINTE
	//#define rem0
	//#define rem1
	//#define rem2
	//#define reml 
	//#define repl //corupted unsorted chunks
	checkGraph(__LINE__);

	// look through intersections dealing with 0, 1 and 2-line intersections
	for (j=g_intersectionsList.m_first, n=0; j!=NULL; j=jnext)
	{
		jnext = j->m_next;
		if (j->m_object.m_n==0)
		{
			#ifndef rem0
			g_intersectionsList.remove(j);
			checkGraph(__LINE__);
			n++;
			#endif
		}
		else if (j->m_object.m_n==1)
		{
			#ifndef rem1
			if (j->m_object.m_lines[0]->m_object.m_i0==j)
				j->m_object.m_lines[0]->m_object.m_i0 = NULL;
			if (j->m_object.m_lines[0]->m_object.m_i1==j)
				j->m_object.m_lines[0]->m_object.m_i1 = NULL;
			g_intersectionsList.remove(j);
			checkGraph(__LINE__);
			n++;
			#endif
		}
		#ifndef rem2
		else if (j->m_object.m_n==2 && 
			j->m_object.m_p.m_x>g_minLineLength && j->m_object.m_p.m_x<LINE_GRID_WIDTH-g_minLineLength && // do not simplify intersections that are
			j->m_object.m_p.m_y>g_minLineLength && j->m_object.m_p.m_y<LINE_GRID_HEIGHT-g_minLineLength) // close to any edge because they don't look like intersections.
		{
			SimpleListNode<Line2> *linen0 = j->m_object.m_lines[0];
			SimpleListNode<Line2> *linen1 = j->m_object.m_lines[1];
			///g_intersectionsList.remove(j);
			if (linen0->m_object.m_i0==j)
			{
				if (linen1->m_object.m_i0==j)
				{
					// this case will lead to an intersection with two of the same line pointers, so get rid of linen1, proceed...
					if (linen0->m_object.m_i1==linen1->m_object.m_i1 && linen1->m_object.m_i1!=NULL)
					{
						#ifndef reml
						linen0->m_object.m_i0 = NULL;
						removeLine(linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
					else
					{
						#ifndef repl
						linen0->m_object.m_p0 = linen1->m_object.m_p1;
						linen0->m_object.m_i0 = linen1->m_object.m_i1;
						replaceLine(linen0, linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
				}
				else
				{
					// this case will lead to an intersection with two of the same line pointers, so get rid of linen1, proceed...
					if (linen0->m_object.m_i1==linen1->m_object.m_i0 && linen1->m_object.m_i0!=NULL)
					{
						#ifdef reml
						linen0->m_object.m_i0 = NULL;
						removeLine(linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
					else
					{
						#ifndef repl
						linen0->m_object.m_p0 = linen1->m_object.m_p0;
						linen0->m_object.m_i0 = linen1->m_object.m_i0;
						replaceLine(linen0, linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
				}
			}
			else
			{
				if (linen1->m_object.m_i0==j)
				{
					// this case will lead to an intersection with two of the same line pointers, so get rid of linen1, proceed...
					if (linen0->m_object.m_i0==linen1->m_object.m_i1 && linen1->m_object.m_i1!=NULL)
					{
						#ifdef reml
						linen0->m_object.m_i1 = NULL;
						removeLine(linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
					else
					{
						#ifndef repl
						linen0->m_object.m_p1 = linen1->m_object.m_p1;
						linen0->m_object.m_i1 = linen1->m_object.m_i1;
						replaceLine(linen0, linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
				}
				else
				{
					// this case will lead to an intersection with two of the same line pointers, so get rid of linen1, proceed...
					if (linen0->m_object.m_i0==linen1->m_object.m_i0 && linen1->m_object.m_i0!=NULL)
					{
						#ifdef reml
						linen0->m_object.m_i1 = NULL;
						removeLine(linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
					else
					{
						#ifndef repl
						linen0->m_object.m_p1 = linen1->m_object.m_p0;
						linen0->m_object.m_i1 = linen1->m_object.m_i0;
						replaceLine(linen0, linen1);
						g_intersectionsList.remove(j);
						n++;
						#endif
					}
				}
			}
			checkGraph(__LINE__);
			//n++;
		}
		#endif
	}
	return n;
}

void cleanIntersections()
{
	uint16_t n;
	
	while(1)
	{
		uint16_t n0 = removeShortLinesIntersections(); 
		uint16_t n1 = removeRedundantLinesIntersections();
		uint16_t n2 = simplifyIntersections();
		#ifdef CIINFO
		printf("CI %d %d %d\n", n0, n1, n2);
		#endif
		if ((n0+n1+n2)==0)
			break;
	}
}

void removeMinLines(uint16_t minLineLength)
{
	SimpleListNode<Line2> *i;
		
	for (i=g_linesList.m_first; i!=NULL; i=i->m_next)
	{
		if (i->m_object.length2()<minLineLength && i->m_object.m_i0==NULL && i->m_object.m_i1==NULL)
			g_linesList.remove(i);
	}
}

line_detect_event_t line_detected=NULL;
void sendTrackedLines(const SimpleList<Tracker<Line2> > &lines, uint8_t renderFlags, const char *desc)
{
	
	SimpleListNode<Tracker<Line2> > *n;
	Line2 *line;
	uint8_t i, mode;
	
	///CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('L','I','S','F')), INT8(RENDER_FLAG_START), STRING(desc), INT16(LINE_GRID_WIDTH), INT16(LINE_GRID_HEIGHT), END);

	for(n=lines.m_first, i=0; n; n=n->m_next, i++)
	{
		mode = n->m_object.get()==NULL ? 1 : 0;
		line = & n->m_object.m_object;
		if(NULL!=line_detected) line_detected((n->m_object.m_index), (line->m_p0.m_x), (line->m_p0.m_y), (line->m_p1.m_x), (line->m_p1.m_y));
		
		#ifdef DTLINE
		printf("TLINE %d %d %d %d %d %d\n", (mode), (n->m_object.m_index), (line->m_p0.m_x), (line->m_p0.m_y), (line->m_p1.m_x), (line->m_p1.m_y));
		#endif
		//CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('L','I','S','S')), INT8(mode), INT8(n->m_object.m_index), INT16(line->m_p0.m_x), INT16(line->m_p0.m_y), INT16(line->m_p1.m_x), INT16(line->m_p1.m_y), END);
	}
	
	//CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('L','I','S','F')), INT8(renderFlags), STRING(desc), INT16(CAM_RES3_WIDTH), INT16(CAM_RES3_HEIGHT), END);	
	
}
	
intersection_detect_event_t intersection_detected=NULL;
void sendIntersections(const SimpleList<Intersection> &lines, uint8_t renderFlags, const char *desc)
{
	SimpleListNode<Intersection> *i;

	///CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('N','A','D','F')), INT8(RENDER_FLAG_START), STRING(desc), INT16(LINE_GRID_WIDTH), INT16(LINE_GRID_HEIGHT), END);
	
	for (i=lines.m_first; i!=NULL; i=i->m_next) {
		if(NULL!=intersection_detected) intersection_detected(i->m_object.m_n, i->m_object.m_p.m_x, i->m_object.m_p.m_y);
		#ifdef DTINTE
		printf("TINTE %d %d %d\n", i->m_object.m_p.m_x, i->m_object.m_p.m_y, i->m_object.m_n);
		#endif
	}
		///CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('N','A','D','S')), INTS8(2, &i->m_object.m_p), END);	
	///CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('N','A','D','F')), INT8(renderFlags), STRING(desc), INT16(LINE_GRID_WIDTH), INT16(LINE_GRID_HEIGHT), END);	

}
Line2 *findLine(uint8_t index)
{
	SimpleListNode<Line2> *i;
	
	for (i=g_linesList.m_first; i!=NULL; i=i->m_next)
	{
		if (i->m_object.m_tracker && i->m_object.m_tracker->m_index==index)
			return &i->m_object;
	}
	return NULL;
}

Line2 *findTrackedLine(uint8_t index)
{
	SimpleListNode<Tracker<Line2> > *i;
	
	for (i=g_lineTrackersList.m_first; i!=NULL; i=i->m_next)
	{
		if (i->m_object.m_index==index)
			return &i->m_object.m_object;
	}
	return NULL;
}
uint8_t trackedLinesWithPoint(const Point &p)
{
	uint8_t n;
	Line2 *line;
	
	SimpleListNode<Tracker<Line2> > *i;
	
	for (i=g_lineTrackersList.m_first, n=0; i!=NULL; i=i->m_next)
	{
		line = &i->m_object.m_object;
		if (line && (line->m_p0.equals(p) || line->m_p1.equals(p))) 
			n++;
	}
	return n;
}

bool compareIntersections(const Intersection &i0, const FrameIntersection &i1)
{
#if 0
	uint8_t i, n;
	
	// count valid lines
	for (i=0, n=0; i<i0.m_n; i++)
	{
		if (i0.m_lines[i]->m_object.m_tracker && i0.m_lines[i]->m_object.m_tracker->get()!=NULL)
			n++;
	}
	
	// are valid lines in i0 equal to number of lines in i1?
	return n==i1.m_n;
#else
	return i0.m_n==i1.m_n;
#endif
}

int16_t getAngle(const Line2 &line, const Point &p)
{
	float x, y;
	int16_t xdiff, ydiff, res;
	
	// find diff's using the point that's not p
	// Note that y axis is pointing down, so we invert.
	if (p.equals(line.m_p0)) 
	{
		xdiff = line.m_p1.m_x - p.m_x;
		ydiff = p.m_y - line.m_p1.m_y;
	}
	else
	{
		xdiff = line.m_p0.m_x - p.m_x;
		ydiff = p.m_y - line.m_p0.m_y;
	}
	// rotate axes 90 degrees so 0 is pointing straight up
	x = ydiff;
	y = -xdiff;
	
	res = (int16_t)(atan2(y, x)*180/(float)M_PI);
	
	return res;
}

int compareAngle(const void *a, const void *b)
{
  return (((FrameIntersectionLine *)a)->m_angle - ((FrameIntersectionLine *)b)->m_angle );
}

void formatIntersection(const Intersection &intersection, FrameIntersection *fintersection, bool tracked)
{
	uint8_t i, j;
		
	for (i=0, j=0; i<intersection.m_n && j<LINE_MAX_FRAME_INTERSECTION_LINES; i++)
	{
		// only add valid lines to fintersection
		if (!tracked || intersection.m_lines[i]->m_object.m_tracker)// && intersection.m_lines[i]->m_object.m_tracker->get()!=NULL))
		{	
			if (tracked)
				fintersection->m_lines[j].m_index = intersection.m_lines[i]->m_object.m_tracker->m_index;
			else
				fintersection->m_lines[j].m_index = 0;				
			fintersection->m_lines[j].m_angle = getAngle(intersection.m_lines[i]->m_object, intersection.m_p);   
			fintersection->m_lines[j].m_reserved = 0;
			j++;
		}
	}
	// zero-out rest of lines
	for (i=j; i<LINE_MAX_FRAME_INTERSECTION_LINES; i++)
	{	
		fintersection->m_lines[i].m_index = 0;
		fintersection->m_lines[i].m_angle = 0;   
		fintersection->m_lines[i].m_reserved = 0;
	}
	
	fintersection->m_x = intersection.m_p.m_x;
	fintersection->m_y = intersection.m_p.m_y;
	fintersection->m_n = j;
	
	// sort lines based on angles
	qsort(fintersection->m_lines, fintersection->m_n, sizeof(FrameIntersectionLine), compareAngle);
}
line_detect_event_t vector_detected = NULL;
void sendPrimaryFeatures(uint8_t renderFlags)
{
	uint8_t n=0;
	Line2 *primary=NULL;
	
	if (g_lineState==LINE_STATE_TRACKING)
	{	
		primary = findTrackedLine(g_primaryLineIndex);
		if (g_primaryIntersection.m_state!=TR_INVALID)
			n = g_primaryIntersection.m_object.m_n;
	}
	if(primary && NULL != vector_detected) vector_detected((n), (primary->m_p0.m_x), (primary->m_p0.m_y), (primary->m_p1.m_x), (primary->m_p1.m_y));
	//if(primary) printf("PRIMARY! %d %d %d %d %d\n", (n), (primary->m_p0.m_x), (primary->m_p0.m_y), (primary->m_p1.m_x), (primary->m_p1.m_y)); else printf("NO PRIMARY!\n");
/*
	if (primary)
	{
		if (g_goalPoint.equals(primary->m_p0))
			CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('P','V','I','0')), INT8(renderFlags), INT16(LINE_GRID_WIDTH), INT16(LINE_GRID_HEIGHT), 
				INT8(primary->m_p1.m_x), INT8(primary->m_p1.m_y), INT8(primary->m_p0.m_x), INT8(primary->m_p0.m_y), INT8(n), END);
		else
			CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('P','V','I','0')), INT8(renderFlags), INT16(LINE_GRID_WIDTH), INT16(LINE_GRID_HEIGHT), 
				INT8(primary->m_p0.m_x), INT8(primary->m_p0.m_y), INT8(primary->m_p1.m_x), INT8(primary->m_p1.m_y), INT8(n), END);
	}
	else // send null message so it always shows up as a layer
		CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('P','V','I','0')), INT8(renderFlags), INT16(LINE_GRID_WIDTH), INT16(LINE_GRID_HEIGHT), 
			INT8(0), INT8(0), INT8(0), INT8(0), INT8(0), END);
	*/	
	
}
int line_hLine(uint8_t row, uint16_t *buf, uint32_t len)
{
	uint16_t j, index, bit0, bit1, col0, col1, lineWidth;

	// copy a lot of code to reduce branching, make it faster
	if (g_whiteLine) // pos neg
	{
		for (j=0; buf[j]<EQ_HSCAN_LINE_START && buf[j+1]<EQ_HSCAN_LINE_START && j<len; j++)
		{
			bit0 = buf[j]&EQ_NEGATIVE;
			bit1 = buf[j+1]&EQ_NEGATIVE;
			col0 = buf[j]&~EQ_NEGATIVE;
			col1 = buf[j+1]&~EQ_NEGATIVE;
			if (bit0==0 && bit1!=0)
			{
				lineWidth = col1 - col0;
				if (g_minLineWidth<lineWidth && lineWidth<g_maxLineWidth)
				{
					index = LINE_GRID_INDEX((((col0+col1)>>1) + g_dist)>>3, row>>1);
					if (index<LINE_GRID_WIDTH*LINE_GRID_HEIGHT+8)
						g_lineGrid[index] |= LINE_NODE_FLAG_HLINE;
					else
						cprintf(0, "high index\n");
				}
			}
		}
	}
	else // black line
	{
 		for (j=0; buf[j]<EQ_HSCAN_LINE_START && buf[j+1]<EQ_HSCAN_LINE_START && j<len; j++)
		{
			bit0 = buf[j]&EQ_NEGATIVE;
			bit1 = buf[j+1]&EQ_NEGATIVE;
			col0 = buf[j]&~EQ_NEGATIVE;
			col1 = buf[j+1]&~EQ_NEGATIVE;
			if (bit0!=0 && bit1==0)
			{
				lineWidth = col1 - col0;
				if (g_minLineWidth<lineWidth && lineWidth<g_maxLineWidth)
				{
					index = LINE_GRID_INDEX((((col0+col1)>>1) + g_dist)>>3, row>>1);
					if (index<LINE_GRID_WIDTH*LINE_GRID_HEIGHT+8)
						g_lineGrid[index] |= LINE_NODE_FLAG_HLINE;
					else
						cprintf(0, "high index\n");
				}
			}
		}
	}
	
	return 0;
}

int line_vLine(uint8_t row, uint8_t *vstate, uint16_t *buf, uint32_t len)
{
	uint16_t i, index, bit0, col0, lineWidth;

	if (g_whiteLine)
	{
		for (i=0; buf[i]<EQ_HSCAN_LINE_START && i<len; i++)
		{
			bit0 = buf[i]&EQ_NEGATIVE;
			col0 = (buf[i]&~EQ_NEGATIVE)>>2;
			if (bit0==0) // pos
				vstate[col0] = row+1;
			else // bit0!=0, neg
			{
				if (vstate[col0]!=0)
				{
					lineWidth = (row - (vstate[col0]-1))<<2; // multiply by 4 because vertical is subsampled by 4
					if (g_minLineWidth<lineWidth && lineWidth<g_maxLineWidth && col0<LINE_VSIZE)
					{
						index = LINE_GRID_INDEX(col0>>1, (row - (lineWidth>>3))>>1);
						if (index<LINE_GRID_WIDTH*LINE_GRID_HEIGHT+8)
							g_lineGrid[index] |= LINE_NODE_FLAG_VLINE;
						else
							cprintf(0, "high index\n");
					}
					vstate[col0] = 0;
				}
			}
		}
	}
	else // black line
	{
		for (i=0; buf[i]<EQ_HSCAN_LINE_START && i<len; i++)
		{
			bit0 = buf[i]&EQ_NEGATIVE;
			col0 = (buf[i]&~EQ_NEGATIVE)>>2;
			if (bit0!=0) // neg
				vstate[col0] = row+1;
			else // bit0==0, pos
			{
				if (vstate[col0]!=0)
				{
					lineWidth = (row - (vstate[col0]-1))<<2; // multiply by 4 because vertical is subsampled by 4
					if (g_minLineWidth<lineWidth && lineWidth<g_maxLineWidth && col0<LINE_VSIZE)
					{
						index = LINE_GRID_INDEX(col0>>1, (row - (lineWidth>>3))>>1);
						if (index<LINE_GRID_WIDTH*LINE_GRID_HEIGHT+8)
							g_lineGrid[index] |= LINE_NODE_FLAG_VLINE;
						else
							cprintf(0, "high index\n");
					}
					vstate[col0] = 0;
				}
			}
		}
	}
	return 0;
}
void clearGrid(const RectB rect)
{
	uint8_t i, j;
	uint16_t index;
	uint16_t height;
	int16_t r0;
	
	height = rect.m_bottom - rect.m_top;
	height *= 4;
	
	
	r0 = rect.m_top-height;
	if (r0<0)
		r0 = 0;
	for (i=r0; i<=rect.m_bottom; i++)
	{
		for (j=rect.m_left, index=i*LINE_GRID_WIDTH+rect.m_left; j<=rect.m_right; j++, index++)
			g_lineGrid[index] = 0;
	}
}

void clearGrid()
{
	// remove edges 
	int16_t val, i;
	uint16_t v, h;
	RectB rect;
	
	for (i=0; i<g_votedBarcodeIndex; i++)
	{
		h = LINE_MMC_HBOUNDARY*g_votedBarcodes[i].m_outline.m_width;
		if (h>>LINE_GRID_WIDTH_REDUCTION==0)
			h = 1<<LINE_GRID_WIDTH_REDUCTION; // ensure minimum of 1 pixel in grid
		v = LINE_MMC_VBOUNDARY*g_votedBarcodes[i].m_outline.m_height;
		if (v>>LINE_GRID_HEIGHT_REDUCTION==0)
			v = 1<<LINE_GRID_HEIGHT_REDUCTION; // ensure minimum of 1 pixel in grid
		
		val = (g_votedBarcodes[i].m_outline.m_xOffset - h)>>LINE_GRID_WIDTH_REDUCTION;
		if (val<0)
			val = 0;
		rect.m_left = val;
		
		val = (g_votedBarcodes[i].m_outline.m_xOffset + g_votedBarcodes[i].m_outline.m_width + h)>>LINE_GRID_WIDTH_REDUCTION;
		if (val>=LINE_GRID_WIDTH)
			val = LINE_GRID_WIDTH-1;
		rect.m_right = val;
		
		val = (g_votedBarcodes[i].m_outline.m_yOffset - v)>>LINE_GRID_HEIGHT_REDUCTION;
		if (val<0)
			val = 0;
		rect.m_top = val;
		
		val = (g_votedBarcodes[i].m_outline.m_yOffset + g_votedBarcodes[i].m_outline.m_height + v)>>LINE_GRID_HEIGHT_REDUCTION;
		if (val>=LINE_GRID_HEIGHT)
			val = LINE_GRID_HEIGHT-1;
		rect.m_bottom = val;
		
		clearGrid(rect); 
	}	
}

uint32_t compareLines(const Line2 &line0, const Line2 &line1)
{
	uint16_t d0, d1, d2, d3, da, db;
	uint32_t angle, df;
	
	d0 = line0.m_p0.dist2(line1.m_p0);
	d1 = line0.m_p0.dist2(line1.m_p1);
	d2 = line0.m_p1.dist2(line1.m_p0);
	d3 = line0.m_p1.dist2(line1.m_p1);
	
	da = MIN(d0, d1);
	db = MIN(d2, d3);
	
	df = da+db;
	
	// compare to max line distance
	if (df>g_maxLineCompare)
		return TR_MAXVAL;

	// test angle
	angle = tanDiffAbs1000(line0.m_p0, line0.m_p1, line1.m_p0, line1.m_p1, true);
	if (angle>g_maxTrackingTanAngle)
		return TR_MAXVAL;
		
	return da+db;
}

uint16_t handleLineTracking2()
{
	uint32_t val, min;
	uint16_t n=0;
	SimpleListNode<Tracker<Line2> > *i;
	SimpleListNode<Line2> *j;
	uint8_t k;
	Line2 *minLine;
	bool priority;
	
	// go through list, find best candidates
	for (i=g_lineTrackersList.m_first; i!=NULL; i=i->m_next)
	{
		// calculate priority -- primary lines and lines in the primary intersection
		priority = false;
		if (g_primaryActive && g_primaryLineIndex==i->m_object.m_index)
			priority = true;
		else if (g_primaryIntersection.m_state!=TR_INVALID)
		{
			for (k=0; k<g_primaryIntersection.m_object.m_n; k++)
			{
				if (g_primaryIntersection.m_object.m_lines[k].m_index==i->m_object.m_index)
				{
					priority = true;
					break;
				}
			}
		}
		// already found minimum, continue
		if (i->m_object.m_minVal!=TR_MAXVAL)
			continue;
		
		// find min
		for (j=g_linesList.m_first, min=TR_MAXVAL, minLine=NULL; j!=NULL; j=j->m_next)
		{
			val = compareLines(i->m_object.m_object, j->m_object);
			if (!priority) 
				val <<= 12; // we scale up by 4096 because this exceeds pretty much all possible compare values due to geometry
			// find minimum, but if line is already chosen, make sure we're a better match
			if (val<min && i->m_object.swappable(val, &j->m_object))
			{
				min = val;
				minLine = &j->m_object;
			}
		}
		if (minLine)
		{
			// if this minimum line already has a tracker, see which is better
			if (minLine->m_tracker)
			{
				minLine->m_tracker->resetMin(); // reset tracker pointed to by current minline
				minLine->m_tracker = &i->m_object; 
				i->m_object.setMin(minLine, min);
				n++;
			}
			else
			{
				minLine->m_tracker = &i->m_object; // update tracker pointer			
				i->m_object.setMin(minLine, min);
				n++;
			}
		}
	}
	return n;
}


void handleLineTracking()
{
	SimpleListNode<Tracker<Line2> > *i, *inext;
	SimpleListNode<Line2> *j;
	uint16_t leading, trailing;
	
	// reset tracking table
	// Note, we don't need to reset g_linesList entries (e.g. m_tracker) because these are renewed  
	for (i=g_lineTrackersList.m_first; i!=NULL; i=i->m_next)
		i->m_object.resetMin();
	
	// do search, find minimums
	while(handleLineTracking2());
	
	// go through, update tracker, remove entries that are no longer valid
	for (i=g_lineTrackersList.m_first; i!=NULL; i=inext)
	{
		inext = i->m_next;
		
		if (i->m_object.update()&TR_EVENT_INVALIDATED)
			g_lineTrackersList.remove(i); // no longer valid?  remove from tracker list
	}	
	
	// find new candidates
	for (j=g_linesList.m_first; j!=NULL; j=j->m_next)
	{
		if (j->m_object.m_tracker==NULL)
		{
			SimpleListNode<Tracker<Line2> > *n;
			leading = g_lineFiltering*LINE_FILTERING_MULTIPLIER;
			trailing = (leading+1)>>1;
			n = g_lineTrackersList.add(Tracker<Line2>(j->m_object, g_lineTrackerIndex++, leading, trailing));
			if (n==NULL)
			{
				cprintf(0, "hlt\n");
				break;
			}
			j->m_object.m_tracker = &n->m_object; // point back to tracker
		}
	}
	///if (g_debug&LINE_DEBUG_TRACKING)
	///{
	///	uint8_t n;
	///	cprintf(0, "Trackers\n");
	///	for (i=g_lineTrackersList.m_first, n=0; i!=NULL; i=i->m_next, n++)
	///		cprintf(0, "   %d: %d %d (%d %d)(%d %d)\n", n, i->m_object.m_index, i->m_object.m_state, 
	///			i->m_object.m_object.m_p0.m_x, i->m_object.m_object.m_p0.m_y, i->m_object.m_object.m_p1.m_x, i->m_object.m_object.m_p1.m_y);
	///}
}



float lineSegError(Point ps[], uint8_t points, uint8_t *maxIndex)
{
	int8_t i, xdiff, ydiff;
	float e, fe, maxe, error, slope;
	
	xdiff = ps[points-1].m_x - ps[0].m_x;
	ydiff = ps[points-1].m_y - ps[0].m_y;
	
	if (ABS(ydiff)>ABS(xdiff)) // vertical
	{
		slope = (float)xdiff/ydiff;
		for (i=1, error=0.0, maxe=-1.0; i<points; i++)
		{
			e = ps[i].m_x - (ps[0].m_x + (ps[i].m_y-ps[0].m_y)*slope);
			fe = fabsf(e);
			if (fe>maxe)
			{
				*maxIndex = i;
				maxe = fe;
			}
			error += e;
		}
	}
	else // horizontal
	{
		slope = (float)ydiff/xdiff;
		for (i=1, error=0.0, maxe=-1.0; i<points; i++)
		{
			e = ps[i].m_y - (ps[0].m_y + (ps[i].m_x-ps[0].m_x)*slope);
			fe = fabsf(e);
			if (fe>maxe)
			{
				*maxIndex = i;
				maxe = fe;
			}
			error += e;
		}
	}

	error = fabsf(error)/(points-1);
	
	return error;
}

	
void cleanGrid(Point ps[], uint8_t points)
{
	uint8_t i;
	int8_t xdiff, ydiff;
	int16_t j;
	
	// don't put bogus indexes in grid
	if (g_lineIndex>=LINE_MAX_LINES)
		return;
	
	xdiff = ps[points-1].m_x - ps[0].m_x;
	ydiff = ps[points-1].m_y - ps[0].m_y;
	
	if (ABS(ydiff)>ABS(xdiff)) // vertical
	{
		for (i=0; i<points-1; i++) // don't clean around the last point, otherwise we might not be able to pick up where we left off
		{
			j = LINE_GRID_INDEX_P(ps[i]);

			g_lineGrid[j] &= ~LINE_NODE_LINE_MASK;
			g_lineGrid[j] |= g_lineIndex;
			if (ps[i].m_x>0)
				g_lineGrid[j-1] |= LINE_NODE_FLAG_NULL;
			if (ps[i].m_x>1)
				g_lineGrid[j-2] |= LINE_NODE_FLAG_NULL;
			if (ps[i].m_x<LINE_GRID_WIDTH-1)
				g_lineGrid[j+1] |= LINE_NODE_FLAG_NULL;
			if (ps[i].m_x<LINE_GRID_WIDTH-2)
				g_lineGrid[j+2] |= LINE_NODE_FLAG_NULL;
		}
	}
	else // horizontal
	{
		for (i=0; i<points-1; i++) // don't clean around the last point, otherwise we might not be able to pick up where we left off
		{
			j = LINE_GRID_INDEX_P(ps[i]);
			
			g_lineGrid[j] &= ~LINE_NODE_LINE_MASK;
			g_lineGrid[j] |= g_lineIndex;
			if (ps[i].m_y>0)
				g_lineGrid[j-LINE_GRID_WIDTH] |= LINE_NODE_FLAG_NULL;
			if (ps[i].m_y>1)
				g_lineGrid[j-LINE_GRID_WIDTH-LINE_GRID_WIDTH] |= LINE_NODE_FLAG_NULL;
			if (ps[i].m_y<LINE_GRID_HEIGHT-1)
				g_lineGrid[j+LINE_GRID_WIDTH] |= LINE_NODE_FLAG_NULL;
			if (ps[i].m_y<LINE_GRID_HEIGHT-2)
				g_lineGrid[j+LINE_GRID_WIDTH+LINE_GRID_WIDTH] |= LINE_NODE_FLAG_NULL;
		}
	}
	// mark last point
	j = LINE_GRID_INDEX_P(ps[i]);
	g_lineGrid[j] &= ~LINE_NODE_LINE_MASK;
	g_lineGrid[j] |= g_lineIndex;
}

int addline(const Point &p0, const Point &p1)
{
	SimpleListNode<Line2> *linen;
	Line2 line;

	if (g_lineIndex>=LINE_MAX_LINES)
		return -2;
	
	line.m_p0 = p0;
	line.m_p1 = p1; 
	line.m_index = g_lineIndex;
	linen = g_linesList.add(line);
	if (linen==NULL)
		return -1;
	g_lines[g_lineIndex++] = linen;	
	g_nodesList.add(p0);
	g_nodesList.add(p1);		
	return 0;
}	

LineSegIndex finishGridNode(Point ps[], uint8_t points, LineSegIndex prevSeg, Point &p0)
{
	float error;
	LineSeg *ls0, *ls1;
	uint8_t maxIndex;
	
	if (points==0 || g_lineSegIndex>=LINE_MAX_SEGMENTS-1 || g_lineIndex>=LINE_MAX_LINES-1)
		return prevSeg; // return prevSeg to help logic in extractLines
	
	// check error of segment, split if necessary
	error = lineSegError(ps, points, &maxIndex);
	if (error<g_maxError || points<=5)
	{
		ls0 = &g_lineSegs[g_lineSegIndex++];
		ls0->reset();
		ls0->m_p0.m_x = ps[0].m_x;
		ls0->m_p0.m_y = ps[0].m_y;
		ls0->m_p1.m_x = ps[points-1].m_x;
		ls0->m_p1.m_y = ps[points-1].m_y;
		ls0->m_ls0 = prevSeg;
		ls0->m_line = g_lineIndex;
		cleanGrid(ps, points);
	}
	else // split 
	{		
		ls0 = &g_lineSegs[g_lineSegIndex++];
		ls0->reset();
		ls0->m_p0.m_x = ps[0].m_x;
		ls0->m_p0.m_y = ps[0].m_y;
		ls0->m_p1.m_x = ps[maxIndex].m_x;
		ls0->m_p1.m_y = ps[maxIndex].m_y;
		ls0->m_ls0 = prevSeg;
		ls0->m_ls1 = g_lineSegIndex;
		ls0->m_line = g_lineIndex;
		cleanGrid(ps, maxIndex+1);

		// split the line
		addline(p0, ps[maxIndex+1]);
		p0 = ps[maxIndex+1];
		
		ls1 = &g_lineSegs[g_lineSegIndex++];
		ls1->reset();
		ls1->m_p0.m_x = ps[maxIndex].m_x;
		ls1->m_p0.m_y = ps[maxIndex].m_y;
		ls1->m_p1.m_x = ps[points-1].m_x;
		ls1->m_p1.m_y = ps[points-1].m_y;
		ls1->m_ls0 = g_lineSegIndex-2;
		ls1->m_line = g_lineIndex;
		cleanGrid(ps+maxIndex+1, points-(maxIndex+1));
	}
	return g_lineSegIndex-1;
}

bool ydirUp(Point &p, uint16_t &i, uint8_t &points, Point ps[])
{
	if (p.m_y==0)
		return false;

	if (g_lineGrid[i-LINE_GRID_WIDTH]&LINE_NODE_FLAG_1 && !(g_lineGrid[i-LINE_GRID_WIDTH]&LINE_NODE_FLAG_NULL))
	{
		p.m_y--;
		ps[points] = p;
		i += -LINE_GRID_WIDTH;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else if (p.m_x>0 && g_lineGrid[i-LINE_GRID_WIDTH-1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i-LINE_GRID_WIDTH-1]&LINE_NODE_FLAG_NULL))
	{
		p.m_y--;
		p.m_x--;
		ps[points] = p;
		i += -LINE_GRID_WIDTH-1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else if (p.m_x<LINE_GRID_WIDTH-1 && g_lineGrid[i-LINE_GRID_WIDTH+1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i-LINE_GRID_WIDTH+1]&LINE_NODE_FLAG_NULL))
	{
		p.m_y--;
		p.m_x++;
		ps[points] = p;
		i += -LINE_GRID_WIDTH+1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else // no points in the up direction
		return false;
}

bool xdirLeft(Point &p, uint16_t &i, uint8_t &points, Point ps[])
{
	if (p.m_x>0 && g_lineGrid[i-1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i-1]&LINE_NODE_FLAG_NULL))
	{
		p.m_x--;
		ps[points] = p;
		i+= -1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else if (p.m_x>0 && p.m_y>0 && g_lineGrid[i-LINE_GRID_WIDTH-1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i-LINE_GRID_WIDTH-1]&LINE_NODE_FLAG_NULL))
	{
		p.m_y--;
		p.m_x--;
		ps[points] = p;
		i += -LINE_GRID_WIDTH-1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else if (p.m_x>0 && p.m_y<LINE_GRID_HEIGHT-1 && g_lineGrid[i+LINE_GRID_WIDTH-1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i+LINE_GRID_WIDTH-1]&LINE_NODE_FLAG_NULL))
	{
		p.m_y++;
		p.m_x--;
		ps[points] = p;
		i += LINE_GRID_WIDTH-1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else
		return false;
}

bool xdirRight(Point &p, uint16_t &i, uint8_t &points, Point ps[])
{
	if (p.m_x<LINE_GRID_WIDTH-1 && g_lineGrid[i+1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i+1]&LINE_NODE_FLAG_NULL))
	{
		p.m_x++;
		ps[points] = p;
		i += 1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else if (p.m_x<LINE_GRID_WIDTH-1 && p.m_y>0 && g_lineGrid[i-LINE_GRID_WIDTH+1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i-LINE_GRID_WIDTH+1]&LINE_NODE_FLAG_NULL))
	{
		p.m_y--;
		p.m_x++;
		ps[points] = p;
		i += -LINE_GRID_WIDTH+1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else if (p.m_x<LINE_GRID_WIDTH-1 && p.m_y<LINE_GRID_HEIGHT-1 && g_lineGrid[i+LINE_GRID_WIDTH+1]&LINE_NODE_FLAG_1 && !(g_lineGrid[i+LINE_GRID_WIDTH+1]&LINE_NODE_FLAG_NULL))
	{
		p.m_y++;
		p.m_x++;
		ps[points] = p;
		i += LINE_GRID_WIDTH+1;
		g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
		points++;
		return true;
	}
	else
		return false;
}

void extractLineSegments(const Point &p)
{
	bool ydir = true;
	bool rdir = true;
	uint16_t i = LINE_GRID_INDEX_P(p);
	Point ps[LINE_MAX_SEGMENT_POINTS];
	uint8_t points = 0;
	LineSegIndex prevSeg = -1, thisSeg = -1;
	uint32_t angle;
	Point p2 = p; // current point	
	Point p0 = p; // first point of line
	
	// nullify current point
	ps[0] = p2;
	g_lineGrid[i] |= LINE_NODE_FLAG_NULL;
	points++;
		
	while(1)
	{
		if (ydir)
		{
			if (!ydirUp(p2, i, points, ps))
			{
				if (!xdirLeft(p2, i, points, ps))
				{
					if (!xdirRight(p2, i, points, ps))
						goto end;
					else
					{
						ydir = false;
						rdir = true;
					}
				}
				else
				{
					ydir = false;
					rdir = false;
				}				
			}
		}
		else if (!ydir && rdir)
		{
			if (!xdirRight(p2, i, points, ps))
			{
				if (!ydirUp(p2, i, points, ps))
					goto end;
				else
					ydir = true;
			}
		}
		else if (!ydir && !rdir)
		{
			if (!xdirLeft(p2, i, points, ps))
			{
				if (!ydirUp(p2, i, points, ps))
					goto end;
				else
					ydir = true;
			}
		}
		
		if (points>=g_pointsPerSeg)
		{
			thisSeg = finishGridNode(ps, points, prevSeg, p0);
			if (prevSeg>=0)
			{
				if (thisSeg-prevSeg>1) // deal with bisected segment 
				{
					g_lineSegs[prevSeg].m_ls1 = thisSeg-1; // link up prev segment with this one
					// look at angle between segments, add if it exceeds a threshold 
					angle = tanDiffAbs1000(g_lineSegs[prevSeg].m_p0, g_lineSegs[prevSeg].m_p1, g_lineSegs[thisSeg-1].m_p0, g_lineSegs[thisSeg-1].m_p1);
				}
				else
				{
					g_lineSegs[prevSeg].m_ls1 = thisSeg; // link up prev segment with this one
					// look at angle between segments, add if it exceeds a threshold 
					angle = tanDiffAbs1000(g_lineSegs[prevSeg].m_p0, g_lineSegs[prevSeg].m_p1, g_lineSegs[thisSeg].m_p0, g_lineSegs[thisSeg].m_p1);
				}
				if (angle>g_maxSegTanAngle)
				{
					// split the line here
					addline(p0, g_lineSegs[prevSeg].m_p1);
					if (g_lineIndex<LINE_MAX_LINES)
					{
						g_lineSegs[thisSeg].m_line = g_lineIndex; // set line seg line index to new index
						cleanGrid(ps, points); // set nodes to new line index
					}
					p0 = ps[0];
				}
			}
			prevSeg = thisSeg;
			points = 0;
		}
	}

	end:
	// should we use the current segment?
	// we should if there were previous segments or if the current segment is bigger than 2
	if (prevSeg>=0 || points>2)
	{		
		thisSeg = finishGridNode(ps, points, prevSeg, p0);	
		if (prevSeg>=0)
			g_lineSegs[prevSeg].m_ls1 = thisSeg;
		prevSeg = thisSeg;
	}
	
	// add line
	if (prevSeg>=0 && g_lineIndex<LINE_MAX_LINES)
	{
		if (addline(p0, g_lineSegs[prevSeg].m_p1)<0)
			return;
	}		
}

void extractLineSegments()
{
	int8_t i, j;
	uint16_t k;
	LineGridNode node;
	
	for (i=LINE_GRID_HEIGHT-1; i>=0; i--) // bottom-up
	{
		k = i*LINE_GRID_WIDTH;
		for (j=0; j<LINE_GRID_WIDTH; j++, k++)
		{
			node = g_lineGrid[k];
			if (node&LINE_NODE_FLAG_1 && !(node&LINE_NODE_FLAG_NULL))
				// we could do some analysis here to find the end of the continuous train of pixels, then asses which direction 
				// the line is headed, upper-right, upper-left if it's horizontal
				extractLineSegments(Point(j, i)); 
		}
	}
}


void addNadir(const Point &p0, const Point &p1)
{
	uint16_t dist;
	SimpleListNode<Nadir> *i;
	Nadir n;
	uint8_t i0, i1, is;
	Point pp0, pp1;
	
	// calc distance between points
	dist = p0.dist2(p1);
	
	// find lines
	i0 = LINE_GRID_LINE_P(p0);
	i1 = LINE_GRID_LINE_P(p1);
	
	if (i0==i1)
		return;
	if (i0<i1)
	{
		pp0 = p0;
		pp1 = p1;
	}
	else // swap
	{
		pp0 = p1;
		pp1 = p0;
		is = i0;
		i0 = i1;
		i1 = is;
	}
	
	// search nadir list for this pair, if it exists, and it's closer, replace it.
	for (i=g_nadirsList.m_first; i!=NULL; i=i->m_next)
	{
		if (i0==LINE_GRID_LINE_P(i->m_object.m_points[0]) && i1==LINE_GRID_LINE_P(i->m_object.m_points[1]))
		{
			if (dist < i->m_object.m_dist)
			{
				i->m_object.m_dist = dist;
				i->m_object.m_points[0] = pp0;
				i->m_object.m_points[1] = pp1;
				i->m_object.m_pavg = pp0;
				i->m_object.m_pavg.avg(pp1);
			}
			return;
		}
	}
			
	// if it isn't in the list, add it.
	n.m_points[0] = pp0;
	n.m_points[1] = pp1;
	n.m_n = 2;
	n.m_dist = dist;
	n.m_pavg = pp0;
	n.m_pavg.avg(pp1);
	g_nadirsList.add(n);
}

void search(const Point &p, uint8_t radius)
{
	int8_t i, r;
	uint16_t j, k;
	uint8_t i0, i1;
	
	j = LINE_GRID_INDEX(p.m_x, p.m_y);
	i0 = g_lineGrid[j]&LINE_NODE_LINE_MASK;
	
	// search up
	r = p.m_y - radius;
	if (r<0) 
		r = 0;
	for (i=p.m_y-1, k=j-LINE_GRID_WIDTH; i>=r; i--, k-=LINE_GRID_WIDTH)
	{
		i1 = g_lineGrid[k]&LINE_NODE_LINE_MASK;
		if (i1==0 || i0==i1)
			continue;
		addNadir(p, Point(p.m_x, i));
	}
	// search down
	r = p.m_y + radius;
	if (r>LINE_GRID_HEIGHT) 
		r = LINE_GRID_HEIGHT;
	for (i=p.m_y+1, k=j+LINE_GRID_WIDTH; i<r; i++, k+=LINE_GRID_WIDTH)
	{
		i1 = g_lineGrid[k]&LINE_NODE_LINE_MASK;
		if (i1==0 || i0==i1)
			continue;
		addNadir(p, Point(p.m_x, i));
	}
	
	// search left
	r = p.m_x - radius;
	if (r<0)
		r = 0;
	for (i=p.m_x-1, k=j-1; i>=r; i--, k--)
	{
		i1 = g_lineGrid[k]&LINE_NODE_LINE_MASK;
		if (i1==0 || i0==i1)
			continue;
		addNadir(p, Point(i, p.m_y));
	}
	// search right
	r = p.m_x + radius;
	if (r>LINE_GRID_WIDTH)
		r = LINE_GRID_WIDTH;
	for (i=p.m_x+1, k=j+1; i<r; i++, k++)
	{
		i1 = g_lineGrid[k]&LINE_NODE_LINE_MASK;
		if (i1==0 || i0==i1)
			continue;
		addNadir(p, Point(i, p.m_y));
	}	
}


void findNadirs()
{
	SimpleListNode<Point> *i, *j;
	uint16_t dist;
	uint8_t li, lj;
	uint16_t maxMergeDist2=g_maxMergeDist*g_maxMergeDist;
	
	// do n^2 search (or n*(n-1)/2 search)
	for (i=g_nodesList.m_first; i!=NULL; i=i->m_next)
	{
		li = LINE_GRID_LINE_P(i->m_object); 
		for (j=i->m_next; j!=NULL; j=j->m_next)
		{
			lj = LINE_GRID_LINE_P(j->m_object); 
			if (li==lj) // if we're the same line, don't bother.  We're looking for a greater line index
				continue;
			dist = i->m_object.dist2(j->m_object);
			if (dist<=maxMergeDist2)
				// in general li<lj because of the ordering of the labeling of the nodes and lines
				// this is important because of the grid search below.  We'll ignore cases where li>=lj
				// in the interest of efficiency.
				addNadir(i->m_object, j->m_object);
		}
	}
	
	// do grid search 
	for (i=g_nodesList.m_first; i!=NULL; i=i->m_next)
		search(i->m_object, g_maxMergeDist);
}

void reduceNadirs()
{
	SimpleListNode<Nadir> *i, *j, *inext, *jnext;
	uint32_t dist;
	uint16_t n, maxMergeDist2=(g_maxMergeDist*g_maxMergeDist*12)>>3; // multiply by a scaling factor of 1.5
	
	while(1)
	{
		// perform n^2 search, looking for nadirs that are close to each other
		n = 0;
		for (i=g_nadirsList.m_first; i!=NULL; i=inext)
		{
			inext = i->m_next;
			for (j=i->m_next; j!=NULL; j=jnext)
			{
				jnext = j->m_next;
				dist = i->m_object.m_pavg.dist2(j->m_object.m_pavg);
				if (dist<maxMergeDist2)
				{
					i->m_object.merge(j->m_object); // merge j -> i
					g_nadirsList.remove(j);
					if (inext==j)
						inext = jnext;
					n++;
				}
			}
		}
		if (n==0) // no change, return...
			break;
	}
}

// line1.m_p0 is center (intersection)
// line1.m_p1 is outer point of line1
// p01 is outer point of line0
void addLine(SimpleListNode<Nadir> *nadirs, uint8_t li0, const Point &p01, SimpleListNode<Line2> *linen1)
{
	SimpleListNode<Intersection> *j;
	uint8_t i, li1 = linen1->m_object.m_index;
	int8_t diffx, diffy;
	bool horiz;
	const Line2 &line1 = linen1->m_object;

	// determine direction
	diffx = p01.m_x - line1.m_p1.m_x;
	diffy = p01.m_y - line1.m_p1.m_y;
	
	horiz = ABS(diffx)>ABS(diffy);

	// look through intersections list for lines that are affected
	for (j=g_intersectionsList.m_first; j!=NULL; j=j->m_next)
	{
		for (i=0; i<j->m_object.m_n; i++)
		{
			// line li0 has changed.  It's had one of its endpoints set to the new intersection
			if (j->m_object.m_lines[i]->m_object.m_index==li0)
			{
				// We only need to look at line1.m_p1 since it's the only point that's an endpoint and therefore possibly coincident with this intersection
				// if line.m_p1 is coincident with the intersection, we need to change the line pointer
				if (j->m_object.m_p.equals(line1.m_p1))
				{
					j->m_object.m_lines[i] = linen1;
					linen1->m_object.m_i1 = j; // if we change the line in an intersection, we need to change the line to point to the intersection
				}
			}
		}
	}

	// look through remaining nadir list for points->lines that are affected
	for (; nadirs!=NULL; nadirs=nadirs->m_next)
	{
		for (i=0; i<nadirs->m_object.m_n; i++)
		{
			const Point &pt = nadirs->m_object.m_points[i];
			//if (LINE_GRID_INDEX_P(pt)>LINE_GRID_HEIGHT*LINE_GRID_WIDTH)
			//	printf("*** %d\n");
			if (li0==LINE_GRID_LINE_P(pt))
			{
				if (horiz)
				{
					if (SIGN(pt.m_x-line1.m_p0.m_x)==SIGN(line1.m_p1.m_x-line1.m_p0.m_x))
					{
						LINE_GRID_P(pt) &= ~LINE_NODE_LINE_MASK;
						LINE_GRID_P(pt) |= li1;
					}
				}
				else // vertical
				{
					if (SIGN(pt.m_y-line1.m_p0.m_y)==SIGN(line1.m_p1.m_y-line1.m_p0.m_y))
					{
						LINE_GRID_P(pt) &= ~LINE_NODE_LINE_MASK;
						LINE_GRID_P(pt) |= li1;
					}
				}
			}			
		}
	}
}

bool breakLine(SimpleListNode<Nadir> *nadirs, uint8_t li, SimpleListNode<Intersection> *intern)
{
	// orig line linen0   p0                m_p               p1
	// broken line        p0 linen0 p1 intersection p0 linen1 p1
	Line2 line2;
	SimpleListNode<Line2> *linen1;
	SimpleListNode<Line2> *linen0 = g_lines[li];
	checkGraph(__LINE__, 1, 2);
	
	if (g_lineIndex>=LINE_MAX_LINES || intern->m_object.m_n>LINE_MAX_INTERSECTION_LINES-2 || 
		linen0->m_object.m_p0.equals(linen0->m_object.m_p1)) // this last case is a crazy case the fouls up the addLine() logic below
		return false;
	
	// create new line, add it to intersection
	line2.m_p1 = linen0->m_object.m_p1;
	line2.m_p0 = intern->m_object.m_p;
	line2.m_index = g_lineIndex;
	linen1 = g_linesList.add(line2);

	if (linen1==NULL)
		return false;

	// modify first line
	linen0->m_object.m_p1 = intern->m_object.m_p;
	intern->m_object.addLine(linen0, intern, 1);

	g_lines[g_lineIndex++] = linen1;
	intern->m_object.addLine(linen1, intern, 0);
	
	// feedforward new line to nadirs we haven't seen yet
	addLine(nadirs, li, linen0->m_object.m_p0, linen1); 
	
	checkGraph(__LINE__, 1, 2);
	
	return true;
}



void formIntersections()
{
	SimpleListNode<Nadir> *i;
	uint8_t j, li;

	checkGraph(__LINE__);
	
	// go through nadir list and break up the lines
	for (i=g_nadirsList.m_first; i!=NULL; i=i->m_next)
	{
		SimpleListNode<Intersection> *intern = new (std::nothrow) SimpleListNode<Intersection>;
		if (intern==NULL)
			return;
		Intersection *inter = &intern->m_object;
		inter->m_p = i->m_object.m_pavg;
		// look at each point/line in the list
		// If it's an endpoint, we extend the line to the intersection point (avg)
		// if it's not endpoint, we need to break the line and form 2 lines, each with the endpoint at the intersection
		
		for (j=inter->m_n=0; j<i->m_object.m_n && inter->m_n<LINE_MAX_INTERSECTION_LINES; j++)
		{
			Line2 *line;
			SimpleListNode<Line2> *linen;
			const Point &pt = i->m_object.m_points[j];
			li = LINE_GRID_LINE_P(pt); 
			if (li==0)
				continue;
			linen = g_lines[li];
			line = &linen->m_object;
			checkGraph(__LINE__, 1, 2, intern);
			if (pt.equals(line->m_p0)) // matches p0
			{
				line->m_p0 = inter->m_p; // extend line
				if (line->m_i0) // if we are already pointing to another intersection, remove and add ours
					removeLine(linen, line->m_i0);
				inter->addLine(linen, intern, 0);
				checkGraph(__LINE__, 0, 0, intern);
				checkGraph(__LINE__, 1, 2);
			}
			else if (pt.equals(line->m_p1)) // matches p1
			{
				line->m_p1 = inter->m_p; // extend line
				if (line->m_i1) // if we are already pointing to another intersection, remove and add ours
					removeLine(linen, line->m_i1);
				inter->addLine(linen, intern, 1);
				checkGraph(__LINE__, 0, 0, intern);
				checkGraph(__LINE__, 1, 2);
			}
			else
			{
				breakLine(i->m_next, li, intern);
				checkGraph(__LINE__, 0, 0, intern);
				checkGraph(__LINE__, 1, 2);
			}
		}
		g_intersectionsList.add(intern);
		checkGraph(__LINE__);
	}
}	

uint8_t updatePrimaryIntersection(SimpleListNode<Intersection> *intern)
{
	FrameIntersection fint;
	
	g_primaryIntersection.resetMin();
	
	if (intern && 
		intern->m_object.m_p.m_y>g_minLineLength*1.4 && // only consider intersections that are sufficiently far from the top
		intern->m_object.m_p.m_y<LINE_GRID_HEIGHT-g_minLineLength*1.4) // and bottom
	{
		if (g_primaryIntersection.m_state==TR_INVALID || compareIntersections(intern->m_object, g_primaryIntersection.m_object))
		{
			formatIntersection(intern->m_object, &fint, true);
			g_primaryIntersection.setMin(&fint, 0);
		}
	}
		
	return g_primaryIntersection.update();
}
void updatePrimaryPoint(const Line2 &primary)
{
	g_primaryActive = primary.m_tracker->get()!=NULL;
	
	if (xdirection(primary.m_p0, primary.m_p1))
	{
		if (g_primaryPointMap&LINE_HT_RIGHT)
		{
			g_primaryPointMap = LINE_HT_RIGHT;
			if (primary.m_p0.m_x > primary.m_p1.m_x) // p0 is the primary point
			{
				g_primaryPoint = primary.m_p0;
				g_goalPoint = primary.m_p1;
				// update UP/DOWN flag
				if (primary.m_p0.m_y > primary.m_p1.m_y)
					g_primaryPointMap |= LINE_HT_DOWN;
			}
			else // p1 is the primary point
			{
				g_primaryPoint = primary.m_p1;
				g_goalPoint = primary.m_p0;
				// update UP/DOWN flag
				if (primary.m_p1.m_y > primary.m_p0.m_y)
					g_primaryPointMap |= LINE_HT_DOWN;
			}
		}
		else // LINE_HT_LEFT
		{
			g_primaryPointMap = LINE_HT_LEFT;
			if (primary.m_p0.m_x < primary.m_p1.m_x) // p0 is the primary point
			{
				g_primaryPoint = primary.m_p0;
				g_goalPoint = primary.m_p1;
				// update UP/DOWN flag
				if (primary.m_p0.m_y > primary.m_p1.m_y)
					g_primaryPointMap |= LINE_HT_DOWN;
			}
			else // p1 is the primary point
			{
				g_primaryPoint = primary.m_p1;
				g_goalPoint = primary.m_p0;
				// update UP/DOWN flag
				if (primary.m_p1.m_y > primary.m_p0.m_y)
					g_primaryPointMap |= LINE_HT_DOWN;
			}
		}				
	}
	else // y direction
	{
		if (g_primaryPointMap&LINE_HT_DOWN)
		{
			g_primaryPointMap = LINE_HT_DOWN;
			if (primary.m_p0.m_y > primary.m_p1.m_y) // p0 is the primary point
			{
				g_primaryPoint = primary.m_p0;
				g_goalPoint = primary.m_p1;
				// update LEFT/RIGHT flag
				if (primary.m_p0.m_x > primary.m_p1.m_x)
					g_primaryPointMap |= LINE_HT_RIGHT;
			}
			else // p1 is the primary point
			{
				g_primaryPoint = primary.m_p1;
				g_goalPoint = primary.m_p0;
				// update LEFT/RIGHT flag
				if (primary.m_p1.m_x > primary.m_p0.m_x)
					g_primaryPointMap |= LINE_HT_RIGHT;
			}
		}
		else // LINE_HT_UP
		{
			g_primaryPointMap = LINE_HT_UP;
			if (primary.m_p0.m_y < primary.m_p1.m_y) // p0 is the primary point
			{
				g_primaryPoint = primary.m_p0;
				g_goalPoint = primary.m_p1;
				// update LEFT/RIGHT flag
				if (primary.m_p0.m_x > primary.m_p1.m_x)
					g_primaryPointMap |= LINE_HT_RIGHT;
			}
			else // p1 is the primary point
			{
				g_primaryPoint = primary.m_p1;
				g_goalPoint = primary.m_p0;
				// update LEFT/RIGHT flag
				if (primary.m_p1.m_x > primary.m_p0.m_x)
					g_primaryPointMap |= LINE_HT_RIGHT;
			}
		}
	}
}

int intersectionTurn()
{
	uint8_t k, mink;
	uint16_t angle, minAngle;
	Line2 *primary;
		
	// find line that best fits our desired turn angle (g_nextTurnAngle), ie, find minimum difference
	for (k=0, minAngle=0xffff; k<g_primaryIntersection.m_object.m_n; k++)
	{
		// calculate difference between angles
		angle = ABS(g_primaryIntersection.m_object.m_lines[k].m_angle - g_nextTurnAngle);
		if (angle<minAngle)
		{
			minAngle = angle;
			mink = k;
		}
	}
	// assign new primary line
	g_primaryLineIndex = g_primaryIntersection.m_object.m_lines[mink].m_index;
	primary = findTrackedLine(g_primaryLineIndex);
	// set g_primaryPointMap
	if (g_primaryIntersection.m_object.m_x==primary->m_p0.m_x && g_primaryIntersection.m_object.m_y==primary->m_p0.m_y) // primary point is p0
	{
		g_primaryPoint = primary->m_p0;
		g_goalPoint = primary->m_p1;
	}
	else // primary point is p1
	{
		g_primaryPoint = primary->m_p1;
		g_goalPoint = primary->m_p0;
	}
	if (xdirection(primary->m_p0, primary->m_p1))
	{
		if (g_primaryPoint.m_x<g_goalPoint.m_x)
			g_primaryPointMap = LINE_HT_LEFT;
		else
			g_primaryPointMap = LINE_HT_RIGHT;							
	}
	else // y direction
	{
		if (g_primaryPoint.m_y<g_goalPoint.m_y)
			g_primaryPointMap = LINE_HT_UP;
		else
			g_primaryPointMap = LINE_HT_DOWN;							
	}
	g_primaryActive = true;
	g_primaryIntersection.m_state = TR_INVALID; // invalidate intersection
	
	// reset angle to default angle
	g_nextTurnAngle = g_defaultTurnAngle;
	g_newTurnAngle = false;
	
	// we always succeed by choosing the closest line
	return 0;	
}

void setPrimaryVector(uint8_t index)
{
	Line2 *line;
	
	line = findTrackedLine(index);
	if (line)
	{
		// choose point lower in the image as the primary point
		g_primaryPointMap = LINE_HT_DOWN;
					
		g_primaryLineIndex = index;
		g_primaryIntersection.m_state = TR_INVALID;
		g_lineState = LINE_STATE_TRACKING;
		// update 
		updatePrimaryPoint(*line);
	}
}

void handleLineState()
{
	uint8_t ymax;
	SimpleListNode<Tracker<Line2> > *i, *max;
	Line2 *line, *primary;
	uint8_t events;
	
	if (g_lineState==LINE_STATE_ACQUIRING)
	{
		if (g_manualVectorSelect) // // wait for user to select
		{
			if (g_manualVectorSelecIndextActive)
			{
				setPrimaryVector(g_manualVectorSelectIndex);
				g_manualVectorSelecIndextActive = false;
			}
			
			return;
		}
		// find line that meets our criteria -- it should be at least 45 degrees past horizontal and reasonably long
		// we will consider lines that are lower in the image first.  The first line to fit the criterial is our line.
		
		// reset 
		for (i=g_lineTrackersList.m_first; i!=NULL; i=i->m_next)
			i->m_object.resetMin();

		while(1)
		{
			for (i=g_lineTrackersList.m_first, ymax=0, max=NULL; i!=NULL; i=i->m_next)
			{
				line = i->m_object.get();
				if (line!=NULL && i->m_object.m_minVal==TR_MAXVAL)
				{
					if (line->m_p0.m_y>ymax)
					{
						ymax = line->m_p0.m_y;
						max = i;
					}
					else if (line->m_p1.m_y>ymax)
					{	
						ymax = line->m_p1.m_y;
						max = i;
					}
				}
			}
			if (max)
			{
				//angle = tanAbs1000(max->m_object.m_object.m_p0, max->m_object.m_object.m_p1);
				//if (angle>=LINE_MIN_ACQUISITION_TAN_ANGLE && max->m_object.m_object.length2()>=LINE_MIN_ACQUISITION_LENGTH2)
				if (max->m_object.m_object.length2()>=LINE_MIN_ACQUISITION_LENGTH2)
				{
					//cprintf("acq %d %d %d %d %d %d\n", max->m_object.m_index, angle, 
					//	max->m_object.m_object.m_p0.m_x, max->m_object.m_object.m_p0.m_y, max->m_object.m_object.m_p1.m_x, max->m_object.m_object.m_p1.m_y);
					
					setPrimaryVector(max->m_object.m_index);
					break;
				}			
				max->m_object.m_minVal = 0; // flag that we've visited
			}
			else
			{
				//cprintf("null\n");
				break;
			}
		}
	}
	else // LINE_STATE_TRACKING
	{
		primary = findTrackedLine(g_primaryLineIndex);
		
		if (primary)
		{
			if (g_reversePrimary)
			{
				g_reversePrimary = false;
				if (g_primaryPointMap&LINE_HT_UP)
				{
					g_primaryPointMap &= ~LINE_HT_UP;
					g_primaryPointMap |= LINE_HT_DOWN;
				}
				else if (g_primaryPointMap&LINE_HT_DOWN)
				{
					g_primaryPointMap &= ~LINE_HT_DOWN;
					g_primaryPointMap |= LINE_HT_UP;
				}
				if (g_primaryPointMap&LINE_HT_RIGHT)
				{
					g_primaryPointMap &= ~LINE_HT_RIGHT;
					g_primaryPointMap |= LINE_HT_LEFT;
				}
				else if (g_primaryPointMap&LINE_HT_LEFT)
				{
					g_primaryPointMap &= ~LINE_HT_LEFT;
					g_primaryPointMap |= LINE_HT_RIGHT;
				}
			}
			// deal with primary point 
			updatePrimaryPoint(*primary);
			
			///if (g_debug&LINE_DEBUG_TRACKING)
			///	cprintf(0, "Primary %d %d (%d %d)(%d %d)\n", g_primaryLineIndex, g_primaryPointMap, 
			///	g_primaryPoint.m_x, g_primaryPoint.m_y, g_goalPoint.m_x, g_goalPoint.m_y);
			
			// Find primary again in list of lines.  Note if we find "primary" here, it will be the same line as "primary" above.  
			// The only difference is this line will always have a valid intersection.  The "otherIndex" and "otherPoint" found 
			// above will apply here also. 
			primary = findLine(g_primaryLineIndex);
			if (primary)
			{
				if (primary->m_p0.equals(g_goalPoint)) // look for intersection at i0
					events = updatePrimaryIntersection(primary->m_i0); 
				else // look for intersection at i1
					events = updatePrimaryIntersection(primary->m_i1);
				
				// as soon as we see the g_primaryIntersection state transition to valid...
				if (events&TR_EVENT_VALIDATED)
				{
					g_newIntersection = true;
					///if (g_debug&LINE_DEBUG_TRACKING)
					///	cprintf(0, "New intersection\n");
					
					// if turn angle is set before intersection, go ahead and choose turn now
					if (!g_delayedTurn)
						intersectionTurn();
					else // LINE_TURNMODE_DURING_INTERSECTION, we need to reset g_newTurnAngle, so we don't accidentally turn
						g_newTurnAngle = false;
				}
				// deal with turn, if turn mode dictates that choose turn during the intersection
				else if (g_newTurnAngle && g_delayedTurn && g_primaryIntersection.m_state!=TR_INVALID)
					intersectionTurn();
					
				if (events&TR_EVENT_INVALIDATED)
					g_newIntersection = false;

				///if ((g_debug&LINE_DEBUG_TRACKING) && g_primaryIntersection.m_state!=TR_INVALID)
				///	cprintf(0, "Intersection %d (%d %d)\n", g_primaryIntersection.m_state, g_primaryIntersection.m_object.m_x, g_primaryIntersection.m_object.m_y);
			}							
		}
		else // we lost the line
			g_lineState = LINE_STATE_ACQUIRING;
		
	}
}

void line_close()
{
	if (g_equeue)
		delete g_equeue;
	if (g_lineBuf)
		free(g_lineBuf);
	if (g_lineGridMem)
		free(g_lineGridMem);
	if (g_lineSegsMem)
		free(g_lineSegsMem);
	if (g_lines)
		free(g_lines);
	if (g_candidateBarcodes)
		free(g_candidateBarcodes);
	if (g_votedBarcodesMem)
		free(g_votedBarcodesMem);
	g_linesList.clear();
	g_nodesList.clear();
	g_nadirsList.clear();
	g_intersectionsList.clear();
	g_lineTrackersList.clear();
	g_barCodeTrackersList.clear();
	line_detected = NULL;
	vector_detected = NULL;
	intersection_detected = NULL;
}

int line_open() ///int8_t progIndex
{
	g_primaryIntersection.setTiming(myleading, mytrailing);
	g_linesList.clear();
	g_nodesList.clear();
	g_nadirsList.clear();
	g_intersectionsList.clear();
	g_lineTrackersList.clear();
	g_barCodeTrackersList.clear();
	
	g_lineGridMem = (uint8_t *)malloc(LINE_GRID_WIDTH*LINE_GRID_HEIGHT*sizeof(LineGridNode)+CAM_PREBUF_LEN+8); // +8 for extra memory at the end because little overruns sometimes happen
	g_lineGrid = (LineGridNode *)(g_lineGridMem+CAM_PREBUF_LEN);
	
	g_lines = (SimpleListNode<Line2> **)malloc(LINE_MAX_LINES*sizeof(SimpleListNode<Line2> *));
	g_lineSegsMem = (uint8_t *)malloc(LINE_MAX_SEGMENTS*sizeof(LineSeg)+CAM_PREBUF_LEN);
	g_lineSegs = (LineSeg *)(g_lineSegsMem+CAM_PREBUF_LEN);
	
	g_candidateBarcodes = (BarCode **)malloc(LINE_MMC_CANDIDATE_BARCODES*sizeof(BarCode *));
	
	g_votedBarcodesMem = (uint8_t *)malloc(LINE_MMC_VOTED_BARCODES*sizeof(DecodedBarCode)+CAM_PREBUF_LEN);
	g_votedBarcodes = (DecodedBarCode *)(g_votedBarcodesMem+CAM_PREBUF_LEN);
	
	g_lineBuf = (uint16_t *)malloc(LINE_BUFSIZE*sizeof(uint16_t)); 
	g_equeue = new (std::nothrow) Equeue;

	g_maxSegTanAngle = tan(M_PI/4)*1000;
	g_maxEquivTanAngle = tan(M_PI/10)*1000;
	g_maxTrackingTanAngle = tan(M_PI/10)*1000;
	g_pointsPerSeg = 12;
	g_maxError = 0.9;
	
	g_barcodeIndex = 0;
	g_votedBarcodeIndex	 = 0;
	g_maxCodeDist = 15*15;
	g_minVotingThreshold = 128; // divide by 256, so 128 is 1/2
	g_barCodeTrackerIndex = 0;
	g_lineTrackerIndex = 0;
	
	g_lineState = LINE_STATE_ACQUIRING;
	g_primaryActive = false;
	g_newIntersection = false;
	g_delayedTurn = false;
	g_defaultTurnAngle = 0;
	g_nextTurnAngle = 0;
	g_newTurnAngle = false;
	g_manualVectorSelect = false;
	g_manualVectorSelecIndextActive = false;

	g_reversePrimary = false;
	
	g_renderMode = LINE_RM_ALL_FEATURES;
	
	if (g_equeue==NULL || g_lineBuf==NULL || g_lineGridMem==NULL || g_lineSegsMem==NULL || 
		g_lines==NULL || g_candidateBarcodes==NULL || g_votedBarcodesMem==NULL)
	{
		cprintf(0, "Line memory error\n");
		line_close();
		return -1;
	}
	
	g_repeat = 0;
	
	g_frameFlag = false;
	
	return 0;///line_loadParams(progIndex);
}

int line_processMain()
{
	static uint32_t n = 0;
	uint32_t i;
	uint32_t len, tlen;
	bool eof, error;
	int8_t row;
	uint8_t vstate[LINE_VSIZE];
	///uint32_t timer;
	SimpleList<uint32_t> timers;
	SimpleListNode<uint32_t> *j;	

	// send frame and data over USB 
	///if (g_renderMode!=LINE_RM_MINIMAL)
	///	cam_sendFrame(g_chirpUsb, CAM_RES3_WIDTH, CAM_RES3_HEIGHT, 
	///		RENDER_FLAG_BLEND, FOURCC('4', '0', '1', '4'));

	// indicate start of edge data
	///if (g_debug&LINE_DEBUG_LAYERS)
	///	CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('E','D','G','F')), HINT8(RENDER_FLAG_START), HINT16(CAM_RES3_WIDTH), HINT16(CAM_RES3_HEIGHT), END);
	
	// initialize variables
	g_lineIndex = 1; // set to 1 because 0 means empty...
	g_lineSegIndex = 0;
	g_barcodeIndex = 0;
	memset(vstate, 0, LINE_VSIZE);
	memset(g_lineGrid, 0, LINE_GRID_WIDTH*LINE_GRID_HEIGHT*sizeof(LineGridNode));
	//printf("INIT\n");
	
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.clear(); // need to clear timers because of repeat flag, leads to memory leak
	
	///if (g_debug&LINE_DEBUG_TRACKING)
	///	cprintf(0, "Frame %d ______\n", n++);
	
	checkGraph(__LINE__);

	if (g_repeat)
		*g_equeue->m_fields = g_savedEqueue;
	else
		g_savedEqueue = *g_equeue->m_fields;
	
	//setTimer(&timer);
	for (i=0, row=-1, tlen=0; true; i++)
	{
		while((len=g_equeue->readLine(g_lineBuf, LINE_BUFSIZE, &eof, &error))==0);
		tlen += len;
		if (g_lineBuf[0]==EQ_HSCAN_LINE_START)
		{
			row++;
			///detectCodes(row, g_lineBuf+1, len-1);
			line_hLine(row, g_lineBuf+1, len-1);
		}
		else if (g_lineBuf[0]==EQ_VSCAN_LINE_START)
			line_vLine(row, vstate, g_lineBuf+1, len-1);

		///if (g_debug&LINE_DEBUG_LAYERS)
		///	CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('E','D','G','S')), UINTS16(len, g_lineBuf), END);
		if (eof || error)
			break;
	}
	//printf("TLEN %d\n", tlen);
		
	outside:
	// indicate end of edge data
	///if (g_debug&LINE_DEBUG_LAYERS)
	///	CRP_SEND_XDATA(g_chirpUsb, HTYPE(FOURCC('E','D','G','F')), 
	///		HINT8(0), HINT16(CAM_RES3_WIDTH), HINT16(CAM_RES3_HEIGHT), END);


	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));

	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	///clusterCodes();
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));

	clearGrid();
	
	g_linesList.clear();
	g_nodesList.clear();
	g_nadirsList.clear();
	g_intersectionsList.clear();
	
	if (error) // deal with error after we call clustercodes otherwise there's a memory leak
	{
		cprintf(0, "error\n");
		g_equeue->flush();
		
		return -1;
	}
		
	g_allMutex = true;
	g_primaryMutex = true;
	///handleBarCodeTracking();
	g_primaryMutex = false;
	
	///if (g_debug&LINE_DEBUG_LAYERS)
	///	sendCodes(0);
	
	///sendTrackedCodes(RENDER_FLAG_BLEND);
	
	///if (g_debug&LINE_DEBUG_LAYERS)
	///	line_sendLineGrid(0);
	
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	extractLineSegments();
	#ifdef DTLINE
	printf("LIDX %d\n", g_lineIndex);
	#endif
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));
	
	///if (g_debug&LINE_DEBUG_LAYERS)
	///{
	///	sendLineSegments(0);
	///	sendPoints(g_nodesList, 0, "nodes");
	///}
	
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	findNadirs();
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));
	
	///if (g_debug&LINE_DEBUG_LAYERS)
	///	sendNadirs(g_nadirsList, 0, "nadir pairs");
	
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	reduceNadirs();
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));

	checkGraph(__LINE__);

	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	formIntersections();
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));

	checkGraph(__LINE__);

	///if (g_debug&LINE_DEBUG_LAYERS)
	///{
	///	sendNadirs(g_nadirsList, 0, "merged nadirs");
	///	sendLines(g_linesList, 0, "pre-cleaned lines");
	///	sendIntersections(g_intersectionsList, 0, "pre-cleaned intersections");
	///}

	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	
	cleanIntersections();///TODO: segmentation 

	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));
		
	checkGraph(__LINE__);

    removeMinLines(g_minLineLength2);
	
	///if (g_debug&LINE_DEBUG_TRACKING)
	///{
	///	SimpleListNode<Line2> *ln;
	///	SimpleListNode<Intersection> *in;
	///	uint32_t k;
	
	///	cprintf(0, "Lines\n");
	///	for (ln=g_linesList.m_first, k=0; ln!=NULL; ln=ln->m_next, k++)
	///		cprintf(0, "  %d: (%d %d %x)(%d %d %x)\n", k, ln->m_object.m_p0.m_x, ln->m_object.m_p0.m_y, ln->m_object.m_i0, ln->m_object.m_p1.m_x, ln->m_object.m_p1.m_y, ln->m_object.m_i1);
	///	cprintf(0, "Intersections\n");
	///	for (in=g_intersectionsList.m_first, k=0; in!=NULL; in=in->m_next, k++)
	///		cprintf(0, " %d: %d (%d %d) %x\n", k, in->m_object.m_n, in->m_object.m_p.m_x, in->m_object.m_p.m_y, in);
	///}
	
	checkGraph(__LINE__);

	///if (g_debug&LINE_DEBUG_LAYERS)
	///	sendLines(g_linesList, 0, "lines");
		
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	setTimer(&timer);
	handleLineTracking();
	g_allMutex = false;
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///	timers.add(getTimer(timer));
	
	if (g_renderMode==LINE_RM_ALL_FEATURES)///|| (g_debug&LINE_DEBUG_LAYERS))
		sendTrackedLines(g_lineTrackersList, RENDER_FLAG_BLEND, "filtered lines");

	if (g_renderMode==LINE_RM_ALL_FEATURES)
		sendIntersections(g_intersectionsList, RENDER_FLAG_BLEND, "intersections");
	///else if (g_debug&LINE_DEBUG_LAYERS)
	///	sendIntersections(g_intersectionsList, 0, "intersections");
	
	g_primaryMutex = true;
	handleLineState();
	g_primaryMutex = false;	
	
	///if (g_debug==LINE_DEBUG_BENCHMARK)
	///{
	///	for (j=timers.m_first, i=0, timer=0; j!=NULL; j=j->m_next, i++)
	///	{
	///		timer += j->m_object;
	///		cprintf(0, "timer %d: %d\n", i, j->m_object);
	///	}
	///	cprintf(0, "total: %d\n", timer);
	///}

	sendPrimaryFeatures(RENDER_FLAG_BLEND);

	// render whatever we've sent
    ///exec_sendEvent(g_chirpUsb, EVT_RENDER_FLUSH);
	
	g_frameFlag = true;
	
	return 0;
}

int line_process()
{
	if (!g_repeat)
	{
		if (g_renderMode!=LINE_RM_MINIMAL)
		{
			SM_OBJECT->currentLine = 0;
			SM_OBJECT->stream = 1; // set streaming

			// wait for frame
			while(SM_OBJECT->currentLine < CAM_RES3_HEIGHT-1)
			{
				// After grab starts, reset streaming so we don't accidentally grab the next frame.
				// If we accidentally grab the next frame, we get overruns in the equeue, etc. 
				// This only happens when we're running PixyMon (streaming over USB) and we're 
				// getting hammered with serial interrupts for communication.  This would sometimes
				// lead to this loop erroring -- we miss currentline and it wraps around.
				if (SM_OBJECT->currentLine>1)
					SM_OBJECT->stream = 0;
			}
		}
		else
			SM_OBJECT->stream = 1; // set streaming					
	}
		
	return line_processMain();
}
/// line.cpp eof

struct EqueueFields *loop2(uint8_t const *d)
{
	//if (SM_OBJECT->stream)
	//	if(0!=grabM0R3((uint8_t *)SRAM1_LOC+CAM_PREBUF_LEN, d)) return 0;
	//printf("grab!\n");
  grabM0R3((uint8_t *)SRAM1_LOC+CAM_PREBUF_LEN, d);
	//printf("proc!\n");
	line_processMain();
	//printf("done!\n");
	return g_Equeue;
}

int line_setVector(uint8_t index)
{
	g_manualVectorSelecIndextActive = true;
	g_manualVectorSelectIndex = index;
	
	return 0;
}

int line_reversePrimary()
{
	// can't reverse primary if it's not active
	if (!g_primaryActive)
		return -1;
	
	g_reversePrimary = true;
	return 0;
}

uint8_t line_shadowCallback(const char *id, uint32_t val)
{
	uint16_t leading, trailing;
	printf("Setting %s to %d\n", id, val);	
	if (strcmp(id, "Edge distance")==0)
	{
		g_dist = *(uint16_t *)&val;
	}
	else if (strcmp(id, "Edge threshold")==0)
	{
		g_thresh = *(uint16_t *)&val;
		g_hThresh = g_thresh*LINE_HTHRESH_RATIO;
	}
	else if (strcmp(id, "Minimum line width")==0)
		g_minLineWidth = *(uint16_t *)&val;
	else if (strcmp(id, "Maximum line width")==0)
		g_maxLineWidth = *(uint16_t *)&val;
	else if (strcmp(id, "Line extraction distance")==0)
		g_extractionDist = *(uint16_t *)&val;
	else if (strcmp(id, "Maximum merge distance")==0)
		g_maxMergeDist = *(uint16_t *)&val;
	else if (strcmp(id, "Minimum line length")==0)
	{
		g_minLineLength = *(uint16_t *)&val;
		g_minLineLength2 = g_minLineLength*g_minLineLength; // squared
	}
	else if (strcmp(id, "Maximum line compare")==0)
		g_maxLineCompare = *(uint32_t *)&val; 
	else if (strcmp(id, "White line")==0)
		g_whiteLine = *(uint8_t *)&val;
	else if (strcmp(id, "Manual vector select")==0)
		g_manualVectorSelect = *(uint8_t *)&val;
	else if (strcmp(id, "Line filtering")==0)
		g_lineFiltering = *(uint8_t *)&val;
	else if (strcmp(id, "Intersection filtering")==0)
	{
		uint8_t v;
		v = *(uint8_t *)&val;
		leading = v*LINE_FILTERING_MULTIPLIER;
		trailing = (leading+1)>>1;
		g_primaryIntersection.setTiming(leading, trailing); 
	}
	else if (strcmp(id, "Barcode filtering")==0)
		g_barcodeFiltering = *(uint8_t *)&val;
	else if (strcmp(id, "Delayed turn")==0)
		g_delayedTurn = *(uint8_t *)&val;
	else if (strcmp(id, "Go")==0)
		g_go = *(uint8_t *)&val;
	else if (strcmp(id, "Repeat")==0)
		g_repeat = *(uint8_t *)&val;
	else if (strcmp(id, "line_setVector")==0)
		line_setVector(*(uint8_t *)&val);
	else if (strcmp(id, "line_reversePrimary")==0)
		line_reversePrimary();
	else if (strcmp(id, "g_maxSegTanAngle")==0) {
		g_maxSegTanAngle = *(uint32_t*)&val;
	} else if (strcmp(id, "g_maxEquivTanAngle")==0) {
		g_maxEquivTanAngle = *(uint32_t*)&val;
	} else if (strcmp(id, "g_maxTrackingTanAngle")==0) {
		g_maxTrackingTanAngle = *(uint32_t*)&val;
	} else if (strcmp(id, "g_pointsPerSeg")==0) {
		g_pointsPerSeg = *(uint8_t*)&val;
	} else {
		return 1;
	}
	return 0;
}
