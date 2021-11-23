// The sketch is auto-generated with XOD (https://xod.io).
//
// You can compile and upload it to an Arduino-compatible board with
// Arduino IDE.
//
// Rough code overview:
//
// - Configuration section
// - STL shim
// - Immutable list classes and functions
// - XOD runtime environment
// - Native node implementation
// - Program graph definition
//
// Search for comments fenced with '====' and '----' to navigate through
// the major code blocks.

#include <Arduino.h>
#include <inttypes.h>


/*=============================================================================
 *
 *
 * Configuration
 *
 *
 =============================================================================*/

// Uncomment to turn on debug of the program
//#define XOD_DEBUG

// Uncomment to trace the program runtime in the Serial Monitor
//#define XOD_DEBUG_ENABLE_TRACE


// Uncomment to make possible simulation of the program
//#define XOD_SIMULATION

#ifdef XOD_SIMULATION
#include <WasmSerial.h>
#define XOD_DEBUG_SERIAL WasmSerial
#else
#define XOD_DEBUG_SERIAL DEBUG_SERIAL
#endif

/*=============================================================================
 *
 *
 * STL shim. Provides implementation for vital std::* constructs
 *
 *
 =============================================================================*/

namespace xod {
namespace std {

template< class T > struct remove_reference      {typedef T type;};
template< class T > struct remove_reference<T&>  {typedef T type;};
template< class T > struct remove_reference<T&&> {typedef T type;};

template <class T>
typename remove_reference<T>::type&& move(T&& a) {
    return static_cast<typename remove_reference<T>::type&&>(a);
}

} // namespace std
} // namespace xod

/*=============================================================================
 *
 *
 * Basic XOD types
 *
 *
 =============================================================================*/
namespace xod {
#if __SIZEOF_FLOAT__ == 4
typedef float Number;
#else
typedef double Number;
#endif
typedef bool Logic;
typedef unsigned long TimeMs;
typedef uint8_t ErrorFlags;

struct Pulse {
    Pulse() {}
    Pulse(bool) {}
    Pulse(int) {}
};

struct XColor {
    constexpr XColor()
        : r(0)
        , g(0)
        , b(0) {}
    constexpr XColor(uint8_t cr, uint8_t cg, uint8_t cb)
        : r(cr)
        , g(cg)
        , b(cb) {}
    uint8_t r, g, b;
};

} // namespace xod

/*=============================================================================
 *
 *
 * XOD-specific list/array implementations
 *
 *
 =============================================================================*/

#ifndef XOD_LIST_H
#define XOD_LIST_H

namespace xod {
namespace detail {

/*
 * Cursors are used internaly by iterators and list views. They are not exposed
 * directly to a list consumer.
 *
 * The base `Cursor` is an interface which provides the bare minimum of methods
 * to facilitate a single iteration pass.
 */
template<typename T> class Cursor {
  public:
    virtual ~Cursor() { }
    virtual bool isValid() const = 0;
    virtual bool value(T* out) const = 0;
    virtual void next() = 0;
};

template<typename T> class NilCursor : public Cursor<T> {
  public:
    virtual bool isValid() const { return false; }
    virtual bool value(T*) const { return false; }
    virtual void next() { }
};

} // namespace detail

/*
 * Iterator is an object used to iterate a list once.
 *
 * Users create new iterators by calling `someList.iterate()`.
 * Iterators are created on stack and are supposed to have a
 * short live, e.g. for a duration of `for` loop or node’s
 * `evaluate` function. Iterators can’t be copied.
 *
 * Implemented as a pimpl pattern wrapper over the cursor.
 * Once created for a cursor, an iterator owns that cursor
 * and will delete the cursor object once destroyed itself.
 */
template<typename T>
class Iterator {
  public:
    static Iterator<T> nil() {
        return Iterator<T>(new detail::NilCursor<T>());
    }

    Iterator(detail::Cursor<T>* cursor)
        : _cursor(cursor)
    { }

    ~Iterator() {
        if (_cursor)
            delete _cursor;
    }

    Iterator(const Iterator& that) = delete;
    Iterator& operator=(const Iterator& that) = delete;

    Iterator(Iterator&& it)
        : _cursor(it._cursor)
    {
        it._cursor = nullptr;
    }

    Iterator& operator=(Iterator&& it) {
        auto tmp = it._cursor;
        it._cursor = _cursor;
        _cursor = tmp;
        return *this;
    }

    operator bool() const { return _cursor->isValid(); }

    bool value(T* out) const {
        return _cursor->value(out);
    }

    T operator*() const {
        T out;
        _cursor->value(&out);
        return out;
    }

    Iterator& operator++() {
        _cursor->next();
        return *this;
    }

  private:
    detail::Cursor<T>* _cursor;
};

/*
 * An interface for a list view. A particular list view provides a new
 * kind of iteration over existing data. This way we can use list slices,
 * list concatenations, list rotations, etc without introducing new data
 * buffers. We just change the way already existing data is iterated.
 *
 * ListView is not exposed to a list user directly, it is used internally
 * by the List class. However, deriving a new ListView is necessary if you
 * make a new list/string processing node.
 */
template<typename T> class ListView {
  public:
    virtual Iterator<T> iterate() const = 0;
};

/*
 * The list as it seen by data consumers. Have a single method `iterate`
 * to create a new iterator.
 *
 * Implemented as pimpl pattern wrapper over a list view. Takes pointer
 * to a list view in constructor and expects the view will be alive for
 * the whole life time of the list.
 */
template<typename T> class List {
  public:
    constexpr List()
        : _view(nullptr)
    { }

    List(const ListView<T>* view)
        : _view(view)
    { }

    Iterator<T> iterate() const {
        return _view ? _view->iterate() : Iterator<T>::nil();
    }

    // pre 0.15.0 backward compatibility
    List* operator->() __attribute__ ((deprecated)) { return this; }
    const List* operator->() const __attribute__ ((deprecated)) { return this; }

  private:
    const ListView<T>* _view;
};

/*
 * A list view over an old good plain C array.
 *
 * Expects the array will be alive for the whole life time of the
 * view.
 */
template<typename T> class PlainListView : public ListView<T> {
  public:
    class Cursor : public detail::Cursor<T> {
      public:
        Cursor(const PlainListView* owner)
            : _owner(owner)
            , _idx(0)
        { }

        bool isValid() const override {
            return _idx < _owner->_len;
        }

        bool value(T* out) const override {
            if (!isValid())
                return false;
            *out = _owner->_data[_idx];
            return true;
        }

        void next() override { ++_idx; }

      private:
        const PlainListView* _owner;
        size_t _idx;
    };

  public:
    PlainListView(const T* data, size_t len)
        : _data(data)
        , _len(len)
    { }

    virtual Iterator<T> iterate() const override {
        return Iterator<T>(new Cursor(this));
    }

  private:
    friend class Cursor;
    const T* _data;
    size_t _len;
};

/*
 * A list view over a null-terminated C-String.
 *
 * Expects the char buffer will be alive for the whole life time of the view.
 * You can use string literals as a buffer, since they are persistent for
 * the program execution time.
 */
class CStringView : public ListView<char> {
  public:
    class Cursor : public detail::Cursor<char> {
      public:
        Cursor(const char* str)
            : _ptr(str)
        { }

        bool isValid() const override {
            return (bool)*_ptr;
        }

        bool value(char* out) const override {
            *out = *_ptr;
            return (bool)*_ptr;
        }

        void next() override { ++_ptr; }

      private:
        const char* _ptr;
    };

  public:
    CStringView(const char* str = nullptr)
        : _str(str)
    { }

    CStringView& operator=(const CStringView& rhs) {
        _str = rhs._str;
        return *this;
    }

    virtual Iterator<char> iterate() const override {
        return _str ? Iterator<char>(new Cursor(_str)) : Iterator<char>::nil();
    }

  private:
    friend class Cursor;
    const char* _str;
};

/*
 * A list view over two other lists (Left and Right) which first iterates the
 * left one, and when exhausted, iterates the right one.
 *
 * Expects both Left and Right to be alive for the whole view life time.
 */
template<typename T> class ConcatListView : public ListView<T> {
  public:
    class Cursor : public detail::Cursor<T> {
      public:
        Cursor(Iterator<T>&& left, Iterator<T>&& right)
            : _left(std::move(left))
            , _right(std::move(right))
        { }

        bool isValid() const override {
            return _left || _right;
        }

        bool value(T* out) const override {
            return _left.value(out) || _right.value(out);
        }

        void next() override {
            _left ? ++_left : ++_right;
        }

      private:
        Iterator<T> _left;
        Iterator<T> _right;
    };

  public:
    ConcatListView() { }

    ConcatListView(List<T> left, List<T> right)
        : _left(left)
        , _right(right)
    { }

    ConcatListView& operator=(const ConcatListView& rhs) {
        _left = rhs._left;
        _right = rhs._right;
        return *this;
    }

    virtual Iterator<T> iterate() const override {
        return Iterator<T>(new Cursor(_left.iterate(), _right.iterate()));
    }

  private:
    friend class Cursor;
    List<T> _left;
    List<T> _right;
};

//----------------------------------------------------------------------------
// Text string helpers
//----------------------------------------------------------------------------

using XString = List<char>;

/*
 * List and list view in a single pack. An utility used to define constant
 * string literals in XOD.
 */
class XStringCString : public XString {
  public:
    XStringCString(const char* str)
        : XString(&_view)
        , _view(str)
    { }

  private:
    CStringView _view;
};

} // namespace xod

#endif

/*=============================================================================
 *
 *
 * Functions to work with memory
 *
 *
 =============================================================================*/

// Define the placement new operator for cores that do not provide their own.
// Note, this definition takes precedence over the existing one (if any). We found no C++ way
// to use the existing implementation _and_ this implementation if not yet defined.
template<typename T>
void* operator new(size_t, T* ptr) noexcept {
    return ptr;
}

/*=============================================================================
 *
 *
 * UART Classes, that wraps Serials
 *
 *
 =============================================================================*/


#if ARDUINO_API_VERSION >= 10001
class arduino::HardwareSerial;
#else
class HardwareSerial;
#endif
class SoftwareSerial;

namespace xod {

class Uart {
  private:
    long _baud;

  protected:
    bool _started = false;

  public:
    Uart(long baud) {
        _baud = baud;
    }

    virtual void begin() = 0;

    virtual void end() = 0;

    virtual void flush() = 0;

    virtual bool available() = 0;

    virtual bool writeByte(uint8_t) = 0;

    virtual bool readByte(uint8_t*) = 0;

    virtual SoftwareSerial* toSoftwareSerial() {
      return nullptr;
    }

    virtual HardwareSerial* toHardwareSerial() {
      return nullptr;
    }

    void changeBaudRate(long baud) {
      _baud = baud;
      if (_started) {
        end();
        begin();
      }
    }

    long getBaudRate() const {
      return _baud;
    }

    Stream* toStream() {
      Stream* stream = (Stream*) toHardwareSerial();
      if (stream) return stream;
      return (Stream*) toSoftwareSerial();
    }
};

class HardwareUart : public Uart {
  private:
    HardwareSerial* _serial;

  public:
    HardwareUart(HardwareSerial& hserial, uint32_t baud = 115200) : Uart(baud) {
      _serial = &hserial;
    }

    void begin();
    void end();
    void flush();

    bool available() {
      return (bool) _serial->available();
    }

    bool writeByte(uint8_t byte) {
      return (bool) _serial->write(byte);
    }

    bool readByte(uint8_t* out) {
      int data = _serial->read();
      if (data == -1) return false;
      *out = data;
      return true;
    }

    HardwareSerial* toHardwareSerial() {
      return _serial;
    }
};

void HardwareUart::begin() {
  _started = true;
  _serial->begin(getBaudRate());
};
void HardwareUart::end() {
  _started = false;
  _serial->end();
};
void HardwareUart::flush() {
  _serial->flush();
};

} // namespace xod

/*=============================================================================
 *
 *
 * Basic algorithms for XOD lists
 *
 *
 =============================================================================*/

#ifndef XOD_LIST_FUNCS_H
#define XOD_LIST_FUNCS_H



namespace xod {

/*
 * Folds a list from left. Also known as "reduce".
 */
template<typename T, typename TR>
TR foldl(List<T> xs, TR (*func)(TR, T), TR acc) {
    for (auto it = xs.iterate(); it; ++it)
        acc = func(acc, *it);
    return acc;
}

template<typename T> size_t lengthReducer(size_t len, T) {
    return len + 1;
}

/*
 * Computes length of a list.
 */
template<typename T> size_t length(List<T> xs) {
    return foldl(xs, lengthReducer<T>, (size_t)0);
}

template<typename T> T* dumpReducer(T* buff, T x) {
    *buff = x;
    return buff + 1;
}

/*
 * Copies a list content into a memory buffer.
 *
 * It is expected that `outBuff` has enough size to fit all the data.
 */
template<typename T> size_t dump(List<T> xs, T* outBuff) {
    T* buffEnd = foldl(xs, dumpReducer, outBuff);
    return buffEnd - outBuff;
}

/*
 * Compares two lists.
 */
template<typename T> bool equal(List<T> lhs, List<T> rhs) {
    auto lhsIt = lhs.iterate();
    auto rhsIt = rhs.iterate();

    for (; lhsIt && rhsIt; ++lhsIt, ++rhsIt) {
        if (*lhsIt != *rhsIt) return false;
    }

    return !lhsIt && !rhsIt;
}

template<typename T> bool operator == (List<T> lhs, List<T> rhs) {
  return equal(lhs, rhs);
}

} // namespace xod

#endif

/*=============================================================================
 *
 *
 * Format Numbers
 *
 *
 =============================================================================*/

/**
 * Provide `formatNumber` cross-platform number to string converter function.
 *
 * Taken from here:
 * https://github.com/client9/stringencoders/blob/master/src/modp_numtoa.c
 * Original function name: `modp_dtoa2`.
 *
 * Modified:
 * - `isnan` instead of tricky comparing and return "NaN"
 * - handle Infinity values and return "Inf" or "-Inf"
 * - return `OVF` and `-OVF` for numbers bigger than max possible, instead of using `sprintf`
 * - use `Number` instead of double
 * - if negative number rounds to zero, return just "0" instead of "-0"
 *
 * This is a replacement of `dtostrf`.
 */

#ifndef XOD_FORMAT_NUMBER_H
#define XOD_FORMAT_NUMBER_H

namespace xod {

/**
 * Powers of 10
 * 10^0 to 10^9
 */
static const Number powers_of_10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000,
    10000000, 100000000, 1000000000 };

static void strreverse(char* begin, char* end) {
    char aux;
    while (end > begin)
        aux = *end, *end-- = *begin, *begin++ = aux;
};

size_t formatNumber(Number value, int prec, char* str) {
    if (isnan(value)) {
        strcpy(str, "NaN");
        return (size_t)3;
    }

    if (isinf(value)) {
        bool isNegative = value < 0;
        strcpy(str, isNegative ? "-Inf" : "Inf");
        return (size_t)isNegative ? 4 : 3;
    }

    /* if input is larger than thres_max return "OVF" */
    const Number thres_max = (Number)(0x7FFFFFFF);

    Number diff = 0.0;
    char* wstr = str;

    if (prec < 0) {
        prec = 0;
    } else if (prec > 9) {
        /* precision of >= 10 can lead to overflow errors */
        prec = 9;
    }

    /* we'll work in positive values and deal with the
     negative sign issue later */
    int neg = 0;
    if (value < 0) {
        neg = 1;
        value = -value;
    }

    uint32_t whole = (uint32_t)value;
    Number tmp = (value - whole) * powers_of_10[prec];
    uint32_t frac = (uint32_t)(tmp);
    diff = tmp - frac;

    if (diff > 0.5) {
        ++frac;
        /* handle rollover, e.g.  case 0.99 with prec 1 is 1.0  */
        if (frac >= powers_of_10[prec]) {
            frac = 0;
            ++whole;
        }
    } else if (diff == 0.5 && prec > 0 && (frac & 1)) {
        /* if halfway, round up if odd, OR
       if last digit is 0.  That last part is strange */
        ++frac;
        if (frac >= powers_of_10[prec]) {
            frac = 0;
            ++whole;
        }
    } else if (diff == 0.5 && prec == 0 && (whole & 1)) {
        ++frac;
        if (frac >= powers_of_10[prec]) {
            frac = 0;
            ++whole;
        }
    }

    if (value > thres_max) {
        if (neg) {
            strcpy(str, "-OVF");
            return (size_t)4;
        }
        strcpy(str, "OVF");
        return (size_t)3;
    }

    int has_decimal = 0;
    int count = prec;
    bool notzero = frac > 0;

    while (count > 0) {
        --count;
        *wstr++ = (char)(48 + (frac % 10));
        frac /= 10;
        has_decimal = 1;
    }

    if (frac > 0) {
        ++whole;
    }

    /* add decimal */
    if (has_decimal) {
        *wstr++ = '.';
    }

    notzero = notzero || whole > 0;

    /* do whole part
   * Take care of sign conversion
   * Number is reversed.
   */
    do
        *wstr++ = (char)(48 + (whole % 10));
    while (whole /= 10);

    if (neg && notzero) {
        *wstr++ = '-';
    }
    *wstr = '\0';
    strreverse(str, wstr - 1);
    return (size_t)(wstr - str);
}

} // namespace xod
#endif


/*=============================================================================
 *
 *
 * Runtime
 *
 *
 =============================================================================*/

//----------------------------------------------------------------------------
// Debug routines
//----------------------------------------------------------------------------
// #ifndef DEBUG_SERIAL
#if defined(XOD_DEBUG) && !defined(DEBUG_SERIAL)
#  define DEBUG_SERIAL Serial
#endif

#if defined(XOD_DEBUG) && defined(XOD_DEBUG_ENABLE_TRACE)
#  define XOD_TRACE(x)      { DEBUG_SERIAL.print(x); DEBUG_SERIAL.flush(); }
#  define XOD_TRACE_LN(x)   { DEBUG_SERIAL.println(x); DEBUG_SERIAL.flush(); }
#  define XOD_TRACE_F(x)    XOD_TRACE(F(x))
#  define XOD_TRACE_FLN(x)  XOD_TRACE_LN(F(x))
#else
#  define XOD_TRACE(x)
#  define XOD_TRACE_LN(x)
#  define XOD_TRACE_F(x)
#  define XOD_TRACE_FLN(x)
#endif

//----------------------------------------------------------------------------
// PGM space utilities
//----------------------------------------------------------------------------
#define pgm_read_nodeid(address) (pgm_read_word(address))

/*
 * Workaround for bugs:
 * https://github.com/arduino/ArduinoCore-sam/pull/43
 * https://github.com/arduino/ArduinoCore-samd/pull/253
 * Remove after the PRs merge
 */
#if !defined(ARDUINO_ARCH_AVR) && defined(pgm_read_ptr)
#  undef pgm_read_ptr
#  define pgm_read_ptr(addr) (*(const void **)(addr))
#endif

namespace xod {
//----------------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------------

TimeMs g_transactionTime;
bool g_isSettingUp;
bool g_isEarlyDeferPass;

//----------------------------------------------------------------------------
// Metaprogramming utilities
//----------------------------------------------------------------------------

template<typename T> struct always_false {
    enum { value = 0 };
};

template<typename T> struct identity {
  typedef T type;
};

template<typename T> struct remove_pointer                    {typedef T type;};
template<typename T> struct remove_pointer<T*>                {typedef T type;};
template<typename T> struct remove_pointer<T* const>          {typedef T type;};
template<typename T> struct remove_pointer<T* volatile>       {typedef T type;};
template<typename T> struct remove_pointer<T* const volatile> {typedef T type;};

template <typename T, typename M> M get_member_type(M T:: *);

//----------------------------------------------------------------------------
// Forward declarations
//----------------------------------------------------------------------------

TimeMs transactionTime();
void runTransaction();

//----------------------------------------------------------------------------
// Engine (private API)
//----------------------------------------------------------------------------

namespace detail {

template<typename NodeT>
bool isTimedOut(const NodeT* node) {
    TimeMs t = node->timeoutAt;
    // TODO: deal with uint32 overflow
    return t && t < transactionTime();
}

template<typename NodeT>
void clearTimeout(NodeT* node) {
    node->timeoutAt = 0;
}

template<typename NodeT>
void clearStaleTimeout(NodeT* node) {
    if (isTimedOut(node))
        clearTimeout(node);
}

void printErrorToDebugSerial(uint16_t nodeId, ErrorFlags errorFlags) {
#if defined(XOD_DEBUG) || defined(XOD_SIMULATION)
    XOD_DEBUG_SERIAL.print(F("+XOD_ERR:"));
    XOD_DEBUG_SERIAL.print(g_transactionTime);
    XOD_DEBUG_SERIAL.print(':');
    XOD_DEBUG_SERIAL.print(nodeId);
    XOD_DEBUG_SERIAL.print(':');
    XOD_DEBUG_SERIAL.print(errorFlags, DEC);
    XOD_DEBUG_SERIAL.print('\r');
    XOD_DEBUG_SERIAL.print('\n');
#endif
}

} // namespace detail

//----------------------------------------------------------------------------
// Public API (can be used by native nodes’ `evaluate` functions)
//----------------------------------------------------------------------------

TimeMs transactionTime() {
    return g_transactionTime;
}

bool isSettingUp() {
    return g_isSettingUp;
}

bool isEarlyDeferPass() {
    return g_isEarlyDeferPass;
}

constexpr bool isValidDigitalPort(uint8_t port) {
#if defined(__AVR__) && defined(NUM_DIGITAL_PINS)
    return port < NUM_DIGITAL_PINS;
#else
    return true;
#endif
}

constexpr bool isValidAnalogPort(uint8_t port) {
#if defined(__AVR__) && defined(NUM_ANALOG_INPUTS)
    return port >= A0 && port < A0 + NUM_ANALOG_INPUTS;
#else
    return true;
#endif
}

} // namespace xod

//----------------------------------------------------------------------------
// Entry point
//----------------------------------------------------------------------------
void setup() {
    // FIXME: looks like there is a rounding bug. Waiting for 100ms fights it
    delay(100);

#if defined(XOD_DEBUG) || defined(XOD_SIMULATION)
    XOD_DEBUG_SERIAL.begin(115200);
    XOD_DEBUG_SERIAL.setTimeout(10);
#endif
    XOD_TRACE_FLN("\n\nProgram started");

    xod::g_isSettingUp = true;
    xod::runTransaction();
    xod::g_isSettingUp = false;
}

void loop() {
    xod::runTransaction();
}

/*=============================================================================
 *
 *
 * Native node implementations
 *
 *
 =============================================================================*/

//-----------------------------------------------------------------------------
// xod/color/color implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__color__color {
struct Node {

    using Type = XColor;

    typedef Type typeof_OUT;

    struct output_OUT { };

    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    typeof_OUT _output_OUT;

    Node (typeof_OUT output_OUT) {
        _output_OUT = output_OUT;
    }

    struct ContextObject {

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                "" \
                " output_OUT");
    }

    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return this->_output_OUT;
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        this->_output_OUT = val;
        ctx->_isOutputDirty_OUT = true;
    }

    void evaluate(Context ctx) {
        Type color = getValue<output_OUT>(ctx);
        emitValue<output_OUT>(ctx, color);
    }

};
} // namespace xod__color__color
} // namespace xod

//-----------------------------------------------------------------------------
// xod-dev/ws2812/ws2812-device implementation
//-----------------------------------------------------------------------------
#include <WS2812.h>

namespace xod {
namespace xod_dev__ws2812__ws2812_device {
template <uint8_t constant_input_PORT>
struct Node {

    typedef uint8_t typeof_PORT;
    typedef Number typeof_W;

    using Type = WS2812*;

    typedef Type typeof_DEV;

    struct input_PORT { };
    struct input_W { };
    struct output_DEV { };

    static const identity<typeof_PORT> getValueType(input_PORT) {
      return identity<typeof_PORT>();
    }
    static const identity<typeof_W> getValueType(input_W) {
      return identity<typeof_W>();
    }
    static const identity<typeof_DEV> getValueType(output_DEV) {
      return identity<typeof_DEV>();
    }

    typeof_DEV _output_DEV;

    Node (typeof_DEV output_DEV) {
        _output_DEV = output_DEV;
    }

    struct ContextObject {

        typeof_W _input_W;

        bool _isOutputDirty_DEV : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_PORT input_W" \
                " output_DEV");
    }

    typeof_PORT getValue(Context ctx, identity<input_PORT>) {
        return constant_input_PORT;
    }
    typeof_W getValue(Context ctx, identity<input_W>) {
        return ctx->_input_W;
    }
    typeof_DEV getValue(Context ctx, identity<output_DEV>) {
        return this->_output_DEV;
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_DEV");
    }

    void emitValue(Context ctx, typeof_DEV val, identity<output_DEV>) {
        this->_output_DEV = val;
        ctx->_isOutputDirty_DEV = true;
    }

    static_assert(isValidDigitalPort(constant_input_PORT), "must be a valid digital port");

    uint8_t mem[sizeof(WS2812)];

    void evaluate(Context ctx) {
        if (!isSettingUp()) return;
        uint32_t w = (uint32_t)getValue<input_W>(ctx);
        Type t = new (mem) WS2812(constant_input_PORT, w);
        emitValue<output_DEV>(ctx, t);
    }

};
} // namespace xod_dev__ws2812__ws2812_device
} // namespace xod

//-----------------------------------------------------------------------------
// xod/uart/uart implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod__uart__uart {
struct Node {

    typedef Number typeof_BAUD;
    typedef Pulse typeof_INIT;

    typedef Pulse typeof_DONE;

    using Type = Uart*;

    typedef Type typeof_UART;

    struct input_BAUD { };
    struct input_INIT { };
    struct output_UART { };
    struct output_DONE { };

    static const identity<typeof_BAUD> getValueType(input_BAUD) {
      return identity<typeof_BAUD>();
    }
    static const identity<typeof_INIT> getValueType(input_INIT) {
      return identity<typeof_INIT>();
    }
    static const identity<typeof_UART> getValueType(output_UART) {
      return identity<typeof_UART>();
    }
    static const identity<typeof_DONE> getValueType(output_DONE) {
      return identity<typeof_DONE>();
    }

    typeof_UART _output_UART;

    Node (typeof_UART output_UART) {
        _output_UART = output_UART;
    }

    struct ContextObject {

        typeof_BAUD _input_BAUD;

        bool _isInputDirty_INIT;

        bool _isOutputDirty_UART : 1;
        bool _isOutputDirty_DONE : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_BAUD input_INIT" \
                " output_UART output_DONE");
    }

    typeof_BAUD getValue(Context ctx, identity<input_BAUD>) {
        return ctx->_input_BAUD;
    }
    typeof_INIT getValue(Context ctx, identity<input_INIT>) {
        return Pulse();
    }
    typeof_UART getValue(Context ctx, identity<output_UART>) {
        return this->_output_UART;
    }
    typeof_DONE getValue(Context ctx, identity<output_DONE>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_INIT");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_INIT>) {
        return ctx->_isInputDirty_INIT;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_UART output_DONE");
    }

    void emitValue(Context ctx, typeof_UART val, identity<output_UART>) {
        this->_output_UART = val;
        ctx->_isOutputDirty_UART = true;
    }
    void emitValue(Context ctx, typeof_DONE val, identity<output_DONE>) {
        ctx->_isOutputDirty_DONE = true;
    }

    uint8_t mem[sizeof(HardwareUart)];
    HardwareUart* uart;

    void evaluate(Context ctx) {
        if (isSettingUp()) {
            auto baud = (uint32_t)getValue<input_BAUD>(ctx);
    #ifdef SERIAL_PORT_HARDWARE_OPEN
            auto serial = SERIAL_PORT_HARDWARE_OPEN;
    #else
            auto serial = SERIAL_PORT_HARDWARE;
    #endif
            uart = new (mem) HardwareUart(serial, baud);
            emitValue<output_UART>(ctx, uart);
        }

        if (isInputDirty<input_INIT>(ctx)) {
            uart->begin();
            emitValue<output_DONE>(ctx, 1);
        }
    }

};
} // namespace xod__uart__uart
} // namespace xod

//-----------------------------------------------------------------------------
// xod/debug/tweak-color implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__debug__tweak_color {
struct Node {

    typedef xod__color__color::Node::Type typeof_OUT;

    struct State {
    };

    struct output_OUT { };

    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    typeof_OUT _output_OUT;

    Node (typeof_OUT output_OUT) {
        _output_OUT = output_OUT;
    }

    struct ContextObject {

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                "" \
                " output_OUT");
    }

    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return this->_output_OUT;
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        this->_output_OUT = val;
        ctx->_isOutputDirty_OUT = true;
    }

    State state;

    State* getState(__attribute__((unused)) Context ctx) {
        return &state;
    }

    void evaluate(Context ctx) {
        // The actual code that makes tweak-nodes work
        // is injected in detail::handleTweaks
    }

};
} // namespace xod__debug__tweak_color
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/continuously implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod__core__continuously {
struct Node {

    typedef Pulse typeof_TICK;

    struct output_TICK { };

    static const identity<typeof_TICK> getValueType(output_TICK) {
      return identity<typeof_TICK>();
    }

    bool isSetImmediate = false;

    Node () {
    }

    struct ContextObject {

        bool _isOutputDirty_TICK : 1;
    };

    using Context = ContextObject*;

    void setImmediate() {
      this->isSetImmediate = true;
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                "" \
                " output_TICK");
    }

    typeof_TICK getValue(Context ctx, identity<output_TICK>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_TICK");
    }

    void emitValue(Context ctx, typeof_TICK val, identity<output_TICK>) {
        ctx->_isOutputDirty_TICK = true;
    }

    void evaluate(Context ctx) {
        emitValue<output_TICK>(ctx, 1);
        setImmediate();
    }

};
} // namespace xod__core__continuously
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/boot implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod__core__boot {
struct Node {

    typedef Pulse typeof_BOOT;

    struct output_BOOT { };

    static const identity<typeof_BOOT> getValueType(output_BOOT) {
      return identity<typeof_BOOT>();
    }

    Node () {
    }

    struct ContextObject {

        bool _isOutputDirty_BOOT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                "" \
                " output_BOOT");
    }

    typeof_BOOT getValue(Context ctx, identity<output_BOOT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_BOOT");
    }

    void emitValue(Context ctx, typeof_BOOT val, identity<output_BOOT>) {
        ctx->_isOutputDirty_BOOT = true;
    }

    void evaluate(Context ctx) {
        emitValue<output_BOOT>(ctx, 1);
    }

};
} // namespace xod__core__boot
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/throttle(color) implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod__core__throttle__color {
struct Node {

    typedef xod__color__color::Node::Type typeof_IN;
    typedef Number typeof_T;

    typedef xod__color__color::Node::Type typeof_OUT;

    struct input_IN { };
    struct input_T { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_T> getValueType(input_T) {
      return identity<typeof_T>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    TimeMs timeoutAt = 0;

    typeof_OUT _output_OUT;

    Node (typeof_OUT output_OUT) {
        _output_OUT = output_OUT;
    }

    struct ContextObject {

        typeof_IN _input_IN;
        typeof_T _input_T;

        bool _isInputDirty_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    void setTimeout(__attribute__((unused)) Context ctx, TimeMs timeout) {
        this->timeoutAt = transactionTime() + timeout;
    }

    void clearTimeout(__attribute__((unused)) Context ctx) {
        detail::clearTimeout(this);
    }

    bool isTimedOut(__attribute__((unused)) const Context ctx) {
        return detail::isTimedOut(this);
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN input_T" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return ctx->_input_IN;
    }
    typeof_T getValue(Context ctx, identity<input_T>) {
        return ctx->_input_T;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return this->_output_OUT;
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_IN");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_IN>) {
        return ctx->_isInputDirty_IN;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        this->_output_OUT = val;
        ctx->_isOutputDirty_OUT = true;
    }

    bool hasStarted : 1;
    bool shouldEmitOnEnd : 1;

    void evaluate(Context ctx) {
        if (isSettingUp()) {
            hasStarted = false;
            shouldEmitOnEnd = false;
        }
        auto inValue = getValue<input_IN>(ctx);

        if (isInputDirty<input_IN>(ctx)) {
            if (!hasStarted) {
                hasStarted = true;
                emitValue<output_OUT>(ctx, inValue);
                TimeMs dt = getValue<input_T>(ctx) * 1000;
                setTimeout(ctx, dt);
            } else {
                shouldEmitOnEnd = true;
            }
        }

        if (isTimedOut(ctx) && shouldEmitOnEnd) {
            shouldEmitOnEnd = false;
            TimeMs dt = getValue<input_T>(ctx) * 1000;
            setTimeout(ctx, dt);
            emitValue<output_OUT>(ctx, inValue);
        } else if (isTimedOut(ctx)) {
            hasStarted = false;
        }
    }

};
} // namespace xod__core__throttle__color
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/pulse-on-change(number) implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__core__pulse_on_change__number {
struct Node {

    typedef Number typeof_IN;

    typedef Pulse typeof_OUT;

    struct input_IN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    Node () {
    }

    struct ContextObject {

        typeof_IN _input_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return ctx->_input_IN;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
    }

    Number sample = NAN;

    void evaluate(Context ctx) {
        auto newValue = getValue<input_IN>(ctx);

        if (!isSettingUp() && newValue != sample)
            emitValue<output_OUT>(ctx, 1);

        sample = newValue;
    }

};
} // namespace xod__core__pulse_on_change__number
} // namespace xod

//-----------------------------------------------------------------------------
// xod/gpio/digital-read-pullup implementation
//-----------------------------------------------------------------------------
//#pragma XOD evaluate_on_pin disable
//#pragma XOD evaluate_on_pin enable input_UPD

namespace xod {
namespace xod__gpio__digital_read_pullup {
template <uint8_t constant_input_PORT>
struct Node {

    typedef uint8_t typeof_PORT;
    typedef Pulse typeof_UPD;

    typedef Logic typeof_SIG;
    typedef Pulse typeof_DONE;

    struct input_PORT { };
    struct input_UPD { };
    struct output_SIG { };
    struct output_DONE { };

    static const identity<typeof_PORT> getValueType(input_PORT) {
      return identity<typeof_PORT>();
    }
    static const identity<typeof_UPD> getValueType(input_UPD) {
      return identity<typeof_UPD>();
    }
    static const identity<typeof_SIG> getValueType(output_SIG) {
      return identity<typeof_SIG>();
    }
    static const identity<typeof_DONE> getValueType(output_DONE) {
      return identity<typeof_DONE>();
    }

    typeof_SIG _output_SIG;

    Node (typeof_SIG output_SIG) {
        _output_SIG = output_SIG;
    }

    struct ContextObject {

        bool _isInputDirty_UPD;

        bool _isOutputDirty_SIG : 1;
        bool _isOutputDirty_DONE : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_PORT input_UPD" \
                " output_SIG output_DONE");
    }

    typeof_PORT getValue(Context ctx, identity<input_PORT>) {
        return constant_input_PORT;
    }
    typeof_UPD getValue(Context ctx, identity<input_UPD>) {
        return Pulse();
    }
    typeof_SIG getValue(Context ctx, identity<output_SIG>) {
        return this->_output_SIG;
    }
    typeof_DONE getValue(Context ctx, identity<output_DONE>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_UPD");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_UPD>) {
        return ctx->_isInputDirty_UPD;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_SIG output_DONE");
    }

    void emitValue(Context ctx, typeof_SIG val, identity<output_SIG>) {
        this->_output_SIG = val;
        ctx->_isOutputDirty_SIG = true;
    }
    void emitValue(Context ctx, typeof_DONE val, identity<output_DONE>) {
        ctx->_isOutputDirty_DONE = true;
    }

    void evaluate(Context ctx) {
        static_assert(isValidDigitalPort(constant_input_PORT), "must be a valid digital port");

        if (!isInputDirty<input_UPD>(ctx))
            return;

        ::pinMode(constant_input_PORT, INPUT_PULLUP);
        emitValue<output_SIG>(ctx, ::digitalRead(constant_input_PORT));
        emitValue<output_DONE>(ctx, 1);
    }

};
} // namespace xod__gpio__digital_read_pullup
} // namespace xod

//-----------------------------------------------------------------------------
// xod/color/pulse-on-change(color) implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__color__pulse_on_change__color {
struct Node {

    typedef xod__color__color::Node::Type typeof_IN;

    typedef Pulse typeof_OUT;

    struct input_IN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    Node () {
    }

    struct ContextObject {

        typeof_IN _input_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return ctx->_input_IN;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
    }

    uint8_t _r = 0;
    uint8_t _g = 0;
    uint8_t _b = 0;

    bool isColorEqual(uint8_t r, uint8_t g, uint8_t b) {
        return (
            _r == r &&
            _g == g &&
            _b == b
        );
    }

    void evaluate(Context ctx) {
        auto newColor = getValue<input_IN>(ctx);

        if (!isSettingUp() && !isColorEqual(newColor.r, newColor.g, newColor.b));
            emitValue<output_OUT>(ctx, 1);

        _r = newColor.r;
        _g = newColor.g;
        _b = newColor.b;
    }

};
} // namespace xod__color__pulse_on_change__color
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/any implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__core__any {
struct Node {

    typedef Pulse typeof_IN1;
    typedef Pulse typeof_IN2;

    typedef Pulse typeof_OUT;

    struct input_IN1 { };
    struct input_IN2 { };
    struct output_OUT { };

    static const identity<typeof_IN1> getValueType(input_IN1) {
      return identity<typeof_IN1>();
    }
    static const identity<typeof_IN2> getValueType(input_IN2) {
      return identity<typeof_IN2>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    Node () {
    }

    struct ContextObject {

        bool _isInputDirty_IN1;
        bool _isInputDirty_IN2;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN1 input_IN2" \
                " output_OUT");
    }

    typeof_IN1 getValue(Context ctx, identity<input_IN1>) {
        return Pulse();
    }
    typeof_IN2 getValue(Context ctx, identity<input_IN2>) {
        return Pulse();
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_IN1 input_IN2");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_IN1>) {
        return ctx->_isInputDirty_IN1;
    }
    bool isInputDirty(Context ctx, identity<input_IN2>) {
        return ctx->_isInputDirty_IN2;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
    }

    void evaluate(Context ctx) {
        bool p1 = isInputDirty<input_IN1>(ctx);
        bool p2 = isInputDirty<input_IN2>(ctx);
        if (p1 || p2)
            emitValue<output_OUT>(ctx, true);
    }

};
} // namespace xod__core__any
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/not implementation
//-----------------------------------------------------------------------------
//#pragma XOD dirtieness disable

namespace xod {
namespace xod__core__not {
struct Node {

    typedef Logic typeof_IN;

    typedef Logic typeof_OUT;

    struct input_IN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    typeof_OUT _output_OUT;

    Node (typeof_OUT output_OUT) {
        _output_OUT = output_OUT;
    }

    struct ContextObject {

        typeof_IN _input_IN;

    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return ctx->_input_IN;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return this->_output_OUT;
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        this->_output_OUT = val;
    }

    void evaluate(Context ctx) {
        auto x = getValue<input_IN>(ctx);
        emitValue<output_OUT>(ctx, !x);
    }

};
} // namespace xod__core__not
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/debounce(boolean) implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod__core__debounce__boolean {
struct Node {

    typedef Logic typeof_ST;
    typedef Number typeof_Ts;

    typedef Logic typeof_OUT;

    struct input_ST { };
    struct input_Ts { };
    struct output_OUT { };

    static const identity<typeof_ST> getValueType(input_ST) {
      return identity<typeof_ST>();
    }
    static const identity<typeof_Ts> getValueType(input_Ts) {
      return identity<typeof_Ts>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    TimeMs timeoutAt = 0;

    typeof_OUT _output_OUT;

    Node (typeof_OUT output_OUT) {
        _output_OUT = output_OUT;
    }

    struct ContextObject {

        typeof_ST _input_ST;
        typeof_Ts _input_Ts;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    void setTimeout(__attribute__((unused)) Context ctx, TimeMs timeout) {
        this->timeoutAt = transactionTime() + timeout;
    }

    void clearTimeout(__attribute__((unused)) Context ctx) {
        detail::clearTimeout(this);
    }

    bool isTimedOut(__attribute__((unused)) const Context ctx) {
        return detail::isTimedOut(this);
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_ST input_Ts" \
                " output_OUT");
    }

    typeof_ST getValue(Context ctx, identity<input_ST>) {
        return ctx->_input_ST;
    }
    typeof_Ts getValue(Context ctx, identity<input_Ts>) {
        return ctx->_input_Ts;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return this->_output_OUT;
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        this->_output_OUT = val;
        ctx->_isOutputDirty_OUT = true;
    }

    bool state = false;

    void evaluate(Context ctx) {
        bool x = getValue<input_ST>(ctx);

        if (x != state) {
            state = x;
            TimeMs dt = getValue<input_Ts>(ctx) * 1000;
            setTimeout(ctx, dt);
        }

        if (isTimedOut(ctx)) {
            emitValue<output_OUT>(ctx, x);
        }
    }

};
} // namespace xod__core__debounce__boolean
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/cast-to-pulse(boolean) implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod__core__cast_to_pulse__boolean {
struct Node {

    typedef Logic typeof_IN;

    typedef Pulse typeof_OUT;

    struct input_IN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    Node () {
    }

    struct ContextObject {

        typeof_IN _input_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return ctx->_input_IN;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
    }

    bool state = false;

    void evaluate(Context ctx) {
        auto newValue = getValue<input_IN>(ctx);

        if (newValue == true && state == false)
            emitValue<output_OUT>(ctx, 1);

        state = newValue;
    }

};
} // namespace xod__core__cast_to_pulse__boolean
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/pulse-on-true implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__core__pulse_on_true {
struct Node {

    typedef Logic typeof_IN;

    typedef Pulse typeof_OUT;

    struct input_IN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    Node () {
    }

    struct ContextObject {

        typeof_IN _input_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return ctx->_input_IN;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                "");
        return false;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
    }

    bool state = false;

    void evaluate(Context ctx) {
        auto newValue = getValue<input_IN>(ctx);

        if (!isSettingUp() && newValue == true && state == false)
            emitValue<output_OUT>(ctx, 1);

        state = newValue;
    }

};
} // namespace xod__core__pulse_on_true
} // namespace xod

//-----------------------------------------------------------------------------
// xod/uart/soft-uart implementation
//-----------------------------------------------------------------------------
#include <SoftwareSerial.h>

namespace xod {
namespace uart_software {
class SoftwareUart : public Uart {
private:
    SoftwareSerial _serial;
    uint8_t _rx;
    uint8_t _tx;

public:
    SoftwareUart(uint8_t rx, uint8_t tx, long baud = 9600)
        : Uart(baud)
        , _serial(rx, tx) {
        _rx = rx;
        _tx = tx;
    }

    void begin();
    void end();
    void flush();

    bool available() {
        return (bool)_serial.available();
    }

    bool writeByte(uint8_t byte) {
        return (bool)_serial.write(byte);
    }

    bool readByte(uint8_t* out) {
        int data = _serial.read();
        if (data == -1)
            return false;
        *out = (uint8_t)data;
        return true;
    }

    uint8_t getRX() {
        return _rx;
    }

    uint8_t getTX() {
        return _tx;
    }

    SoftwareSerial* toSoftwareSerial() {
        return &_serial;
    }
};

void SoftwareUart::begin() {
    _started = true;
    _serial.begin(getBaudRate());
};
void SoftwareUart::end() {
    _started = false;
    _serial.end();
};
void SoftwareUart::flush() {
    _serial.flush();
}

} // namespace uart_software
} // namespace xod

namespace xod {
namespace xod__uart__soft_uart {
template <uint8_t constant_input_RX, uint8_t constant_input_TX>
struct Node {

    typedef uint8_t typeof_RX;
    typedef uint8_t typeof_TX;
    typedef Number typeof_BAUD;
    typedef Pulse typeof_INIT;

    typedef xod__uart__uart::Node::Type typeof_UART;
    typedef Pulse typeof_DONE;

    struct input_RX { };
    struct input_TX { };
    struct input_BAUD { };
    struct input_INIT { };
    struct output_UART { };
    struct output_DONE { };

    static const identity<typeof_RX> getValueType(input_RX) {
      return identity<typeof_RX>();
    }
    static const identity<typeof_TX> getValueType(input_TX) {
      return identity<typeof_TX>();
    }
    static const identity<typeof_BAUD> getValueType(input_BAUD) {
      return identity<typeof_BAUD>();
    }
    static const identity<typeof_INIT> getValueType(input_INIT) {
      return identity<typeof_INIT>();
    }
    static const identity<typeof_UART> getValueType(output_UART) {
      return identity<typeof_UART>();
    }
    static const identity<typeof_DONE> getValueType(output_DONE) {
      return identity<typeof_DONE>();
    }

    typeof_UART _output_UART;

    Node (typeof_UART output_UART) {
        _output_UART = output_UART;
    }

    struct ContextObject {

        typeof_BAUD _input_BAUD;

        bool _isInputDirty_INIT;

        bool _isOutputDirty_UART : 1;
        bool _isOutputDirty_DONE : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_RX input_TX input_BAUD input_INIT" \
                " output_UART output_DONE");
    }

    typeof_RX getValue(Context ctx, identity<input_RX>) {
        return constant_input_RX;
    }
    typeof_TX getValue(Context ctx, identity<input_TX>) {
        return constant_input_TX;
    }
    typeof_BAUD getValue(Context ctx, identity<input_BAUD>) {
        return ctx->_input_BAUD;
    }
    typeof_INIT getValue(Context ctx, identity<input_INIT>) {
        return Pulse();
    }
    typeof_UART getValue(Context ctx, identity<output_UART>) {
        return this->_output_UART;
    }
    typeof_DONE getValue(Context ctx, identity<output_DONE>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_INIT");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_INIT>) {
        return ctx->_isInputDirty_INIT;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_UART output_DONE");
    }

    void emitValue(Context ctx, typeof_UART val, identity<output_UART>) {
        this->_output_UART = val;
        ctx->_isOutputDirty_UART = true;
    }
    void emitValue(Context ctx, typeof_DONE val, identity<output_DONE>) {
        ctx->_isOutputDirty_DONE = true;
    }

    uint8_t mem[sizeof(uart_software::SoftwareUart)];
    uart_software::SoftwareUart* uart;

    void evaluate(Context ctx) {
        if (isSettingUp()) {
            uint8_t rx = getValue<input_RX>(ctx);
            uint8_t tx = getValue<input_TX>(ctx);
            long baud = (long)getValue<input_BAUD>(ctx);
            uart = new (mem) uart_software::SoftwareUart(rx, tx, baud);
            emitValue<output_UART>(ctx, uart);
        }

        if (isInputDirty<input_INIT>(ctx)) {
            uart->begin();
            emitValue<output_DONE>(ctx, 1);
        }
    }

};
} // namespace xod__uart__soft_uart
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/gate(pulse) implementation
//-----------------------------------------------------------------------------

namespace xod {
namespace xod__core__gate__pulse {
struct Node {

    typedef Pulse typeof_IN;
    typedef Logic typeof_EN;

    typedef Pulse typeof_OUT;

    struct input_IN { };
    struct input_EN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_EN> getValueType(input_EN) {
      return identity<typeof_EN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    Node () {
    }

    struct ContextObject {

        typeof_EN _input_EN;

        bool _isInputDirty_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN input_EN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return Pulse();
    }
    typeof_EN getValue(Context ctx, identity<input_EN>) {
        return ctx->_input_EN;
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_IN");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_IN>) {
        return ctx->_isInputDirty_IN;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
    }

    void evaluate(Context ctx) {
        if (getValue<input_EN>(ctx) && isInputDirty<input_IN>(ctx))
            emitValue<output_OUT>(ctx, true);
    }

};
} // namespace xod__core__gate__pulse
} // namespace xod

//-----------------------------------------------------------------------------
// xod/uart/write-byte implementation
//-----------------------------------------------------------------------------
//#pragma XOD evaluate_on_pin disable
//#pragma XOD evaluate_on_pin enable input_SEND
//#pragma XOD error_raise enable

namespace xod {
namespace xod__uart__write_byte {
struct Node {

    typedef xod__uart__uart::Node::Type typeof_UART;
    typedef uint8_t typeof_BYTE;
    typedef Pulse typeof_SEND;

    typedef Pulse typeof_DONE;

    struct input_UART { };
    struct input_BYTE { };
    struct input_SEND { };
    struct output_DONE { };

    static const identity<typeof_UART> getValueType(input_UART) {
      return identity<typeof_UART>();
    }
    static const identity<typeof_BYTE> getValueType(input_BYTE) {
      return identity<typeof_BYTE>();
    }
    static const identity<typeof_SEND> getValueType(input_SEND) {
      return identity<typeof_SEND>();
    }
    static const identity<typeof_DONE> getValueType(output_DONE) {
      return identity<typeof_DONE>();
    }

    union NodeErrors {
        struct {
            bool output_DONE : 1;
        };

        ErrorFlags flags = 0;
    };

    NodeErrors errors = {};

    Node () {
    }

    struct ContextObject {

        typeof_UART _input_UART;
        typeof_BYTE _input_BYTE;

        bool _isInputDirty_SEND;

        bool _isOutputDirty_DONE : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_UART input_BYTE input_SEND" \
                " output_DONE");
    }

    typeof_UART getValue(Context ctx, identity<input_UART>) {
        return ctx->_input_UART;
    }
    typeof_BYTE getValue(Context ctx, identity<input_BYTE>) {
        return ctx->_input_BYTE;
    }
    typeof_SEND getValue(Context ctx, identity<input_SEND>) {
        return Pulse();
    }
    typeof_DONE getValue(Context ctx, identity<output_DONE>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_SEND");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_SEND>) {
        return ctx->_isInputDirty_SEND;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_DONE");
    }

    void emitValue(Context ctx, typeof_DONE val, identity<output_DONE>) {
        ctx->_isOutputDirty_DONE = true;
        this->errors.output_DONE = false;
    }

    template<typename OutputT> void raiseError(Context ctx) {
        raiseError(ctx, identity<OutputT>());
    }

    template<typename OutputT> void raiseError(Context ctx, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_DONE");
    }

    void raiseError(Context ctx, identity<output_DONE>) {
        this->errors.output_DONE = true;
        ctx->_isOutputDirty_DONE = true;
    }

    void raiseError(Context ctx) {
        this->errors.output_DONE = true;
        ctx->_isOutputDirty_DONE = true;
    }

    void evaluate(Context ctx) {
        if (!isInputDirty<input_SEND>(ctx))
            return;

        auto uart = getValue<input_UART>(ctx);
        uint8_t byte = getValue<input_BYTE>(ctx);
        bool res = uart->writeByte(byte);
        if (res) {
            emitValue<output_DONE>(ctx, 1);
        } else {
            raiseError(ctx); // No bytes written
        }
    }

};
} // namespace xod__uart__write_byte
} // namespace xod

//-----------------------------------------------------------------------------
// xod-dev/ws2812/fill-solid implementation
//-----------------------------------------------------------------------------
namespace xod {
namespace xod_dev__ws2812__fill_solid {
template <typename typeof_DEV>
struct Node {

    typedef xod__color__color::Node::Type typeof_C;
    typedef Number typeof_NUM;
    typedef Pulse typeof_DO;

    typedef typeof_DEV typeof_DEVU0027;
    typedef Pulse typeof_DONE;

    struct input_DEV { };
    struct input_C { };
    struct input_NUM { };
    struct input_DO { };
    struct output_DEVU0027 { };
    struct output_DONE { };

    static const identity<typeof_DEV> getValueType(input_DEV) {
      return identity<typeof_DEV>();
    }
    static const identity<typeof_C> getValueType(input_C) {
      return identity<typeof_C>();
    }
    static const identity<typeof_NUM> getValueType(input_NUM) {
      return identity<typeof_NUM>();
    }
    static const identity<typeof_DO> getValueType(input_DO) {
      return identity<typeof_DO>();
    }
    static const identity<typeof_DEVU0027> getValueType(output_DEVU0027) {
      return identity<typeof_DEVU0027>();
    }
    static const identity<typeof_DONE> getValueType(output_DONE) {
      return identity<typeof_DONE>();
    }

    typeof_DEVU0027 _output_DEVU0027;

    Node (typeof_DEVU0027 output_DEVU0027) {
        _output_DEVU0027 = output_DEVU0027;
    }

    struct ContextObject {

        typeof_DEV _input_DEV;
        typeof_C _input_C;
        typeof_NUM _input_NUM;

        bool _isInputDirty_DO;

        bool _isOutputDirty_DEVU0027 : 1;
        bool _isOutputDirty_DONE : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_DEV input_C input_NUM input_DO" \
                " output_DEVU0027 output_DONE");
    }

    typeof_DEV getValue(Context ctx, identity<input_DEV>) {
        return ctx->_input_DEV;
    }
    typeof_C getValue(Context ctx, identity<input_C>) {
        return ctx->_input_C;
    }
    typeof_NUM getValue(Context ctx, identity<input_NUM>) {
        return ctx->_input_NUM;
    }
    typeof_DO getValue(Context ctx, identity<input_DO>) {
        return Pulse();
    }
    typeof_DEVU0027 getValue(Context ctx, identity<output_DEVU0027>) {
        return this->_output_DEVU0027;
    }
    typeof_DONE getValue(Context ctx, identity<output_DONE>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_DO");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_DO>) {
        return ctx->_isInputDirty_DO;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_DEVU0027 output_DONE");
    }

    void emitValue(Context ctx, typeof_DEVU0027 val, identity<output_DEVU0027>) {
        this->_output_DEVU0027 = val;
        ctx->_isOutputDirty_DEVU0027 = true;
    }
    void emitValue(Context ctx, typeof_DONE val, identity<output_DONE>) {
        ctx->_isOutputDirty_DONE = true;
    }

    void evaluate(Context ctx) {
        auto dev = getValue<input_DEV>(ctx);
        if (isInputDirty<input_DO>(ctx)) {
            auto color = getValue<input_C>(ctx);
            auto num = getValue<input_NUM>(ctx);
            if (isinf(num)) {
                // Fill all the LEDs
                dev->fill(color);
            } else {
                // Fill only specified number of LEDs
                bool fromTail = num < 0;
                uint32_t pixelsCount = (uint32_t) abs(num);
                dev->fill(color, pixelsCount, fromTail);
            }

            emitValue<output_DONE>(ctx, 1);
        }
        emitValue<output_DEVU0027>(ctx, dev);
    }

};
} // namespace xod_dev__ws2812__fill_solid
} // namespace xod

//-----------------------------------------------------------------------------
// xod/uart/end implementation
//-----------------------------------------------------------------------------
//#pragma XOD evaluate_on_pin disable
//#pragma XOD evaluate_on_pin enable input_UPD

namespace xod {
namespace xod__uart__end {
struct Node {

    typedef xod__uart__uart::Node::Type typeof_UART;
    typedef Pulse typeof_UPD;

    typedef Pulse typeof_DONE;

    struct input_UART { };
    struct input_UPD { };
    struct output_DONE { };

    static const identity<typeof_UART> getValueType(input_UART) {
      return identity<typeof_UART>();
    }
    static const identity<typeof_UPD> getValueType(input_UPD) {
      return identity<typeof_UPD>();
    }
    static const identity<typeof_DONE> getValueType(output_DONE) {
      return identity<typeof_DONE>();
    }

    Node () {
    }

    struct ContextObject {

        typeof_UART _input_UART;

        bool _isInputDirty_UPD;

        bool _isOutputDirty_DONE : 1;
    };

    using Context = ContextObject*;

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_UART input_UPD" \
                " output_DONE");
    }

    typeof_UART getValue(Context ctx, identity<input_UART>) {
        return ctx->_input_UART;
    }
    typeof_UPD getValue(Context ctx, identity<input_UPD>) {
        return Pulse();
    }
    typeof_DONE getValue(Context ctx, identity<output_DONE>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_UPD");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_UPD>) {
        return ctx->_isInputDirty_UPD;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_DONE");
    }

    void emitValue(Context ctx, typeof_DONE val, identity<output_DONE>) {
        ctx->_isOutputDirty_DONE = true;
    }

    void evaluate(Context ctx) {
        if (!isInputDirty<input_UPD>(ctx))
            return;

        auto uart = getValue<input_UART>(ctx);
        uart->end();
        emitValue<output_DONE>(ctx, 1);
    }

};
} // namespace xod__uart__end
} // namespace xod

//-----------------------------------------------------------------------------
// xod/core/defer(pulse) implementation
//-----------------------------------------------------------------------------
//#pragma XOD error_catch enable
//#pragma XOD error_raise enable

namespace xod {
namespace xod__core__defer__pulse {
struct Node {

    typedef Pulse typeof_IN;

    typedef Pulse typeof_OUT;

    struct input_IN { };
    struct output_OUT { };

    static const identity<typeof_IN> getValueType(input_IN) {
      return identity<typeof_IN>();
    }
    static const identity<typeof_OUT> getValueType(output_OUT) {
      return identity<typeof_OUT>();
    }

    union NodeErrors {
        struct {
            bool output_OUT : 1;
        };

        ErrorFlags flags = 0;
    };

    NodeErrors errors = {};
    bool isSetImmediate = false;

    Node () {
    }

    struct ContextObject {
        uint8_t _error_input_IN;

        bool _isInputDirty_IN;

        bool _isOutputDirty_OUT : 1;
    };

    using Context = ContextObject*;

    void setImmediate() {
      this->isSetImmediate = true;
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx) {
        return getValue(ctx, identity<PinT>());
    }

    template<typename PinT> typename decltype(getValueType(PinT()))::type getValue(Context ctx, identity<PinT>) {
        static_assert(always_false<PinT>::value,
                "Invalid pin descriptor. Expected one of:" \
                " input_IN" \
                " output_OUT");
    }

    typeof_IN getValue(Context ctx, identity<input_IN>) {
        return Pulse();
    }
    typeof_OUT getValue(Context ctx, identity<output_OUT>) {
        return Pulse();
    }

    template<typename InputT> bool isInputDirty(Context ctx) {
        return isInputDirty(ctx, identity<InputT>());
    }

    template<typename InputT> bool isInputDirty(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_IN");
        return false;
    }

    bool isInputDirty(Context ctx, identity<input_IN>) {
        return ctx->_isInputDirty_IN;
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val) {
        emitValue(ctx, val, identity<OutputT>());
    }

    template<typename OutputT> void emitValue(Context ctx, typename decltype(getValueType(OutputT()))::type val, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void emitValue(Context ctx, typeof_OUT val, identity<output_OUT>) {
        ctx->_isOutputDirty_OUT = true;
        if (isEarlyDeferPass()) this->errors.output_OUT = false;
    }

    template<typename OutputT> void raiseError(Context ctx) {
        raiseError(ctx, identity<OutputT>());
    }

    template<typename OutputT> void raiseError(Context ctx, identity<OutputT>) {
        static_assert(always_false<OutputT>::value,
                "Invalid output descriptor. Expected one of:" \
                " output_OUT");
    }

    void raiseError(Context ctx, identity<output_OUT>) {
        this->errors.output_OUT = true;
        ctx->_isOutputDirty_OUT = true;
    }

    void raiseError(Context ctx) {
        this->errors.output_OUT = true;
        ctx->_isOutputDirty_OUT = true;
    }

    template<typename InputT> uint8_t getError(Context ctx) {
        return getError(ctx, identity<InputT>());
    }

    template<typename InputT> uint8_t getError(Context ctx, identity<InputT>) {
        static_assert(always_false<InputT>::value,
                "Invalid input descriptor. Expected one of:" \
                " input_IN");
        return 0;
    }

    uint8_t getError(Context ctx, identity<input_IN>) {
        return ctx->_error_input_IN;
    }

    bool shouldRaiseAtTheNextDeferOnlyRun = false;
    bool shouldPulseAtTheNextDeferOnlyRun = false;

    void evaluate(Context ctx) {
        if (isEarlyDeferPass()) {
            if (shouldRaiseAtTheNextDeferOnlyRun) {
                raiseError<output_OUT>(ctx);
                shouldRaiseAtTheNextDeferOnlyRun = false;
            }

            if (shouldPulseAtTheNextDeferOnlyRun) {
                emitValue<output_OUT>(ctx, true);
                shouldPulseAtTheNextDeferOnlyRun = false;
            }
        } else {
            if (getError<input_IN>(ctx)) {
                shouldRaiseAtTheNextDeferOnlyRun = true;
            } else if (isInputDirty<input_IN>(ctx)) {
                shouldPulseAtTheNextDeferOnlyRun = true;
            }

            setImmediate();
        }
    }

};
} // namespace xod__core__defer__pulse
} // namespace xod
;


/*=============================================================================
 *
 *
 * Main loop components
 *
 *
 =============================================================================*/

namespace xod {

// Define/allocate persistent storages (state, timeout, output data) for all nodes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

// outputs of constant nodes
constexpr Number node_10_output_VAL = 0.02;
constexpr uint8_t node_11_output_VAL = A6;
constexpr Number node_12_output_VAL = 0.02;
constexpr uint8_t node_13_output_VAL = A7;
constexpr Number node_14_output_VAL = 0.02;
constexpr uint8_t node_15_output_VAL = A8;
constexpr Number node_16_output_VAL = 0.02;
constexpr uint8_t node_17_output_VAL = A9;
constexpr Number node_18_output_VAL = 0.02;
constexpr uint8_t node_19_output_VAL = 4;
constexpr Number node_20_output_VAL = 0.02;
constexpr uint8_t node_21_output_VAL = 5;
constexpr uint8_t node_22_output_VAL = 0x6;
constexpr uint8_t node_23_output_VAL = 19;
constexpr uint8_t node_24_output_VAL = 18;
constexpr Number node_25_output_VAL = 9600;
constexpr uint8_t node_26_output_VAL = 0x7E;
constexpr uint8_t node_27_output_VAL = 0xEF;
constexpr uint8_t node_28_output_VAL = 0xFF;
constexpr uint8_t node_29_output_VAL = 0x0;
constexpr uint8_t node_30_output_VAL = 0x3;
constexpr uint8_t node_31_output_VAL = 0x0;
constexpr uint8_t node_32_output_VAL = 0x1;
constexpr uint8_t node_33_output_VAL = 0x6;
constexpr uint8_t node_34_output_VAL = 19;
constexpr uint8_t node_35_output_VAL = 18;
constexpr Number node_36_output_VAL = 9600;
constexpr uint8_t node_37_output_VAL = 0x7E;
constexpr uint8_t node_38_output_VAL = 0xEF;
constexpr uint8_t node_39_output_VAL = 0xFF;
constexpr uint8_t node_40_output_VAL = 0x0;
constexpr uint8_t node_41_output_VAL = 0x3;
constexpr uint8_t node_42_output_VAL = 0x0;
constexpr uint8_t node_43_output_VAL = 0x2;
constexpr uint8_t node_44_output_VAL = 0x6;
constexpr uint8_t node_45_output_VAL = 19;
constexpr uint8_t node_46_output_VAL = 18;
constexpr Number node_47_output_VAL = 9600;
constexpr uint8_t node_48_output_VAL = 0x7E;
constexpr uint8_t node_49_output_VAL = 0xEF;
constexpr uint8_t node_50_output_VAL = 0xFF;
constexpr uint8_t node_51_output_VAL = 0x0;
constexpr uint8_t node_52_output_VAL = 0x3;
constexpr uint8_t node_53_output_VAL = 0x0;
constexpr uint8_t node_54_output_VAL = 0x3;
constexpr uint8_t node_55_output_VAL = 0x6;
constexpr uint8_t node_56_output_VAL = 19;
constexpr uint8_t node_57_output_VAL = 18;
constexpr Number node_58_output_VAL = 9600;
constexpr uint8_t node_59_output_VAL = 0x7E;
constexpr uint8_t node_60_output_VAL = 0xEF;
constexpr uint8_t node_61_output_VAL = 0xFF;
constexpr uint8_t node_62_output_VAL = 0x0;
constexpr uint8_t node_63_output_VAL = 0x3;
constexpr uint8_t node_64_output_VAL = 0x0;
constexpr uint8_t node_65_output_VAL = 0x4;
constexpr uint8_t node_66_output_VAL = 0x6;
constexpr uint8_t node_67_output_VAL = 19;
constexpr uint8_t node_68_output_VAL = 18;
constexpr Number node_69_output_VAL = 9600;
constexpr uint8_t node_70_output_VAL = 0x7E;
constexpr uint8_t node_71_output_VAL = 0xEF;
constexpr uint8_t node_72_output_VAL = 0xFF;
constexpr uint8_t node_73_output_VAL = 0x0;
constexpr uint8_t node_74_output_VAL = 0x3;
constexpr uint8_t node_75_output_VAL = 0x0;
constexpr uint8_t node_76_output_VAL = 0x3;
constexpr uint8_t node_77_output_VAL = 0x6;
constexpr uint8_t node_78_output_VAL = 19;
constexpr uint8_t node_79_output_VAL = 18;
constexpr Number node_80_output_VAL = 9600;
constexpr uint8_t node_81_output_VAL = 0x7E;
constexpr uint8_t node_82_output_VAL = 0xEF;
constexpr uint8_t node_83_output_VAL = 0xFF;
constexpr uint8_t node_84_output_VAL = 0x0;
constexpr uint8_t node_85_output_VAL = 0x3;
constexpr uint8_t node_86_output_VAL = 0x0;
constexpr uint8_t node_87_output_VAL = 0x1;
constexpr uint8_t node_88_output_VAL = 10;
constexpr Number node_89_output_VAL = 16;
constexpr Number node_90_output_VAL = 0.05;
constexpr Number node_91_output_VAL = INFINITY;
constexpr Number node_92_output_VAL = 16;
constexpr uint8_t node_93_output_VAL = 10;
constexpr Number node_94_output_VAL = 16;
constexpr Number node_95_output_VAL = 0.05;
constexpr Number node_96_output_VAL = INFINITY;
constexpr Number node_97_output_VAL = 16;
constexpr uint8_t node_98_output_VAL = 10;
constexpr Number node_99_output_VAL = 16;
constexpr Number node_100_output_VAL = 0.05;
constexpr Number node_101_output_VAL = INFINITY;
constexpr Number node_102_output_VAL = 16;
constexpr uint8_t node_103_output_VAL = 10;
constexpr Number node_104_output_VAL = 16;
constexpr Number node_105_output_VAL = 0.05;
constexpr Number node_106_output_VAL = INFINITY;
constexpr Number node_107_output_VAL = 16;
constexpr uint8_t node_108_output_VAL = 10;
constexpr Number node_109_output_VAL = 16;
constexpr Number node_110_output_VAL = 0.05;
constexpr Number node_111_output_VAL = INFINITY;
constexpr Number node_112_output_VAL = 16;
constexpr uint8_t node_113_output_VAL = 10;
constexpr Number node_114_output_VAL = 16;
constexpr Number node_115_output_VAL = 0.05;
constexpr Number node_116_output_VAL = INFINITY;
constexpr Number node_117_output_VAL = 16;
constexpr Number node_118_output_VAL = 0.02;
constexpr uint8_t node_119_output_VAL = 2;
constexpr uint8_t node_120_output_VAL = 0x6;
constexpr uint8_t node_121_output_VAL = 19;
constexpr uint8_t node_122_output_VAL = 18;
constexpr Number node_123_output_VAL = 9600;
constexpr uint8_t node_124_output_VAL = 0x7E;
constexpr uint8_t node_125_output_VAL = 0xEF;
constexpr uint8_t node_126_output_VAL = 0xFF;
constexpr uint8_t node_127_output_VAL = 0x0;
constexpr uint8_t node_128_output_VAL = 0x3;
constexpr uint8_t node_129_output_VAL = 0x0;
constexpr uint8_t node_130_output_VAL = 0x6;
constexpr uint8_t node_131_output_VAL = 10;
constexpr Number node_132_output_VAL = 16;
constexpr Number node_133_output_VAL = 0.05;
constexpr Number node_134_output_VAL = INFINITY;
constexpr Number node_135_output_VAL = 16;
constexpr Number node_136_output_VAL = 0.02;
constexpr uint8_t node_137_output_VAL = 3;
constexpr uint8_t node_138_output_VAL = 0x6;
constexpr uint8_t node_139_output_VAL = 19;
constexpr uint8_t node_140_output_VAL = 18;
constexpr Number node_141_output_VAL = 9600;
constexpr uint8_t node_142_output_VAL = 0x7E;
constexpr uint8_t node_143_output_VAL = 0xEF;
constexpr uint8_t node_144_output_VAL = 0xFF;
constexpr uint8_t node_145_output_VAL = 0x0;
constexpr uint8_t node_146_output_VAL = 0x3;
constexpr uint8_t node_147_output_VAL = 0x0;
constexpr uint8_t node_148_output_VAL = 0x4;
constexpr uint8_t node_149_output_VAL = 10;
constexpr Number node_150_output_VAL = 16;
constexpr Number node_151_output_VAL = 0.05;
constexpr Number node_152_output_VAL = INFINITY;
constexpr Number node_153_output_VAL = 16;
constexpr Number node_154_output_VAL = 0.02;
constexpr uint8_t node_155_output_VAL = 9;
constexpr uint8_t node_156_output_VAL = 0x6;
constexpr uint8_t node_157_output_VAL = 19;
constexpr uint8_t node_158_output_VAL = 18;
constexpr Number node_159_output_VAL = 9600;
constexpr uint8_t node_160_output_VAL = 0x7E;
constexpr uint8_t node_161_output_VAL = 0xEF;
constexpr uint8_t node_162_output_VAL = 0xFF;
constexpr uint8_t node_163_output_VAL = 0x0;
constexpr uint8_t node_164_output_VAL = 0x3;
constexpr uint8_t node_165_output_VAL = 0x0;
constexpr uint8_t node_166_output_VAL = 0x4;
constexpr uint8_t node_167_output_VAL = 10;
constexpr Number node_168_output_VAL = 16;
constexpr Number node_169_output_VAL = 0.05;
constexpr Number node_170_output_VAL = INFINITY;
constexpr Number node_171_output_VAL = 16;
constexpr Number node_172_output_VAL = 0.02;
constexpr uint8_t node_173_output_VAL = 11;
constexpr uint8_t node_174_output_VAL = 0x6;
constexpr uint8_t node_175_output_VAL = 19;
constexpr uint8_t node_176_output_VAL = 18;
constexpr Number node_177_output_VAL = 9600;
constexpr uint8_t node_178_output_VAL = 0x7E;
constexpr uint8_t node_179_output_VAL = 0xEF;
constexpr uint8_t node_180_output_VAL = 0xFF;
constexpr uint8_t node_181_output_VAL = 0x0;
constexpr uint8_t node_182_output_VAL = 0x3;
constexpr uint8_t node_183_output_VAL = 0x0;
constexpr uint8_t node_184_output_VAL = 0x4;
constexpr uint8_t node_185_output_VAL = 10;
constexpr Number node_186_output_VAL = 16;
constexpr Number node_187_output_VAL = 0.05;
constexpr Number node_188_output_VAL = INFINITY;
constexpr Number node_189_output_VAL = 16;

#pragma GCC diagnostic pop

struct TransactionState {
    bool node_0_isNodeDirty : 1;
    bool node_0_isOutputDirty_OUT : 1;
    bool node_1_isNodeDirty : 1;
    bool node_1_isOutputDirty_OUT : 1;
    bool node_2_isNodeDirty : 1;
    bool node_2_isOutputDirty_OUT : 1;
    bool node_3_isNodeDirty : 1;
    bool node_3_isOutputDirty_OUT : 1;
    bool node_4_isNodeDirty : 1;
    bool node_4_isOutputDirty_OUT : 1;
    bool node_5_isNodeDirty : 1;
    bool node_5_isOutputDirty_OUT : 1;
    bool node_6_isNodeDirty : 1;
    bool node_6_isOutputDirty_OUT : 1;
    bool node_7_isNodeDirty : 1;
    bool node_7_isOutputDirty_OUT : 1;
    bool node_8_isNodeDirty : 1;
    bool node_8_isOutputDirty_OUT : 1;
    bool node_9_isNodeDirty : 1;
    bool node_9_isOutputDirty_OUT : 1;
    bool node_190_isNodeDirty : 1;
    bool node_190_isOutputDirty_TICK : 1;
    bool node_191_isNodeDirty : 1;
    bool node_191_isOutputDirty_BOOT : 1;
    bool node_192_isNodeDirty : 1;
    bool node_192_isOutputDirty_DEV : 1;
    bool node_193_isNodeDirty : 1;
    bool node_193_isOutputDirty_OUT : 1;
    bool node_194_isNodeDirty : 1;
    bool node_194_isOutputDirty_OUT : 1;
    bool node_195_isNodeDirty : 1;
    bool node_195_isOutputDirty_DEV : 1;
    bool node_196_isNodeDirty : 1;
    bool node_196_isOutputDirty_OUT : 1;
    bool node_197_isNodeDirty : 1;
    bool node_197_isOutputDirty_OUT : 1;
    bool node_198_isNodeDirty : 1;
    bool node_198_isOutputDirty_DEV : 1;
    bool node_199_isNodeDirty : 1;
    bool node_199_isOutputDirty_OUT : 1;
    bool node_200_isNodeDirty : 1;
    bool node_200_isOutputDirty_OUT : 1;
    bool node_201_isNodeDirty : 1;
    bool node_201_isOutputDirty_DEV : 1;
    bool node_202_isNodeDirty : 1;
    bool node_202_isOutputDirty_OUT : 1;
    bool node_203_isNodeDirty : 1;
    bool node_203_isOutputDirty_OUT : 1;
    bool node_204_isNodeDirty : 1;
    bool node_204_isOutputDirty_DEV : 1;
    bool node_205_isNodeDirty : 1;
    bool node_205_isOutputDirty_OUT : 1;
    bool node_206_isNodeDirty : 1;
    bool node_206_isOutputDirty_OUT : 1;
    bool node_207_isNodeDirty : 1;
    bool node_207_isOutputDirty_DEV : 1;
    bool node_208_isNodeDirty : 1;
    bool node_208_isOutputDirty_OUT : 1;
    bool node_209_isNodeDirty : 1;
    bool node_209_isOutputDirty_OUT : 1;
    bool node_210_isNodeDirty : 1;
    bool node_210_isOutputDirty_DEV : 1;
    bool node_211_isNodeDirty : 1;
    bool node_211_isOutputDirty_OUT : 1;
    bool node_212_isNodeDirty : 1;
    bool node_212_isOutputDirty_OUT : 1;
    bool node_213_isNodeDirty : 1;
    bool node_213_isOutputDirty_DEV : 1;
    bool node_214_isNodeDirty : 1;
    bool node_214_isOutputDirty_OUT : 1;
    bool node_215_isNodeDirty : 1;
    bool node_215_isOutputDirty_OUT : 1;
    bool node_216_isNodeDirty : 1;
    bool node_216_isOutputDirty_DEV : 1;
    bool node_217_isNodeDirty : 1;
    bool node_217_isOutputDirty_OUT : 1;
    bool node_218_isNodeDirty : 1;
    bool node_218_isOutputDirty_OUT : 1;
    bool node_219_isNodeDirty : 1;
    bool node_219_isOutputDirty_DEV : 1;
    bool node_220_isNodeDirty : 1;
    bool node_220_isOutputDirty_OUT : 1;
    bool node_221_isNodeDirty : 1;
    bool node_221_isOutputDirty_OUT : 1;
    bool node_222_isNodeDirty : 1;
    bool node_222_isOutputDirty_SIG : 1;
    bool node_223_isNodeDirty : 1;
    bool node_223_isOutputDirty_SIG : 1;
    bool node_224_isNodeDirty : 1;
    bool node_224_isOutputDirty_SIG : 1;
    bool node_225_isNodeDirty : 1;
    bool node_225_isOutputDirty_SIG : 1;
    bool node_226_isNodeDirty : 1;
    bool node_226_isOutputDirty_SIG : 1;
    bool node_227_isNodeDirty : 1;
    bool node_227_isOutputDirty_SIG : 1;
    bool node_228_isNodeDirty : 1;
    bool node_228_isOutputDirty_SIG : 1;
    bool node_229_isNodeDirty : 1;
    bool node_229_isOutputDirty_SIG : 1;
    bool node_230_isNodeDirty : 1;
    bool node_230_isOutputDirty_SIG : 1;
    bool node_231_isNodeDirty : 1;
    bool node_231_isOutputDirty_SIG : 1;
    bool node_232_isNodeDirty : 1;
    bool node_232_isOutputDirty_OUT : 1;
    bool node_233_isNodeDirty : 1;
    bool node_233_isOutputDirty_OUT : 1;
    bool node_234_isNodeDirty : 1;
    bool node_234_isOutputDirty_OUT : 1;
    bool node_235_isNodeDirty : 1;
    bool node_235_isOutputDirty_OUT : 1;
    bool node_236_isNodeDirty : 1;
    bool node_236_isOutputDirty_OUT : 1;
    bool node_237_isNodeDirty : 1;
    bool node_237_isOutputDirty_OUT : 1;
    bool node_238_isNodeDirty : 1;
    bool node_238_isOutputDirty_OUT : 1;
    bool node_239_isNodeDirty : 1;
    bool node_239_isOutputDirty_OUT : 1;
    bool node_240_isNodeDirty : 1;
    bool node_240_isOutputDirty_OUT : 1;
    bool node_241_isNodeDirty : 1;
    bool node_241_isOutputDirty_OUT : 1;
    bool node_242_isNodeDirty : 1;
    bool node_242_isOutputDirty_OUT : 1;
    bool node_243_isNodeDirty : 1;
    bool node_243_isOutputDirty_OUT : 1;
    bool node_244_isNodeDirty : 1;
    bool node_244_isOutputDirty_OUT : 1;
    bool node_245_isNodeDirty : 1;
    bool node_245_isOutputDirty_OUT : 1;
    bool node_246_isNodeDirty : 1;
    bool node_246_isOutputDirty_OUT : 1;
    bool node_247_isNodeDirty : 1;
    bool node_247_isOutputDirty_OUT : 1;
    bool node_248_isNodeDirty : 1;
    bool node_248_isOutputDirty_OUT : 1;
    bool node_249_isNodeDirty : 1;
    bool node_249_isOutputDirty_OUT : 1;
    bool node_250_isNodeDirty : 1;
    bool node_250_isOutputDirty_OUT : 1;
    bool node_251_isNodeDirty : 1;
    bool node_251_isOutputDirty_OUT : 1;
    bool node_252_isNodeDirty : 1;
    bool node_252_isOutputDirty_OUT : 1;
    bool node_253_isNodeDirty : 1;
    bool node_253_isOutputDirty_OUT : 1;
    bool node_254_isNodeDirty : 1;
    bool node_254_isOutputDirty_OUT : 1;
    bool node_255_isNodeDirty : 1;
    bool node_255_isOutputDirty_OUT : 1;
    bool node_256_isNodeDirty : 1;
    bool node_256_isOutputDirty_OUT : 1;
    bool node_257_isNodeDirty : 1;
    bool node_257_isOutputDirty_OUT : 1;
    bool node_258_isNodeDirty : 1;
    bool node_258_isOutputDirty_OUT : 1;
    bool node_259_isNodeDirty : 1;
    bool node_259_isOutputDirty_OUT : 1;
    bool node_260_isNodeDirty : 1;
    bool node_260_isOutputDirty_OUT : 1;
    bool node_261_isNodeDirty : 1;
    bool node_261_isOutputDirty_OUT : 1;
    bool node_262_isNodeDirty : 1;
    bool node_262_isOutputDirty_OUT : 1;
    bool node_263_isNodeDirty : 1;
    bool node_263_isOutputDirty_OUT : 1;
    bool node_264_isNodeDirty : 1;
    bool node_264_isOutputDirty_OUT : 1;
    bool node_265_isNodeDirty : 1;
    bool node_265_isOutputDirty_OUT : 1;
    bool node_266_isNodeDirty : 1;
    bool node_266_isOutputDirty_OUT : 1;
    bool node_267_isNodeDirty : 1;
    bool node_267_isOutputDirty_OUT : 1;
    bool node_268_isNodeDirty : 1;
    bool node_268_isOutputDirty_OUT : 1;
    bool node_269_isNodeDirty : 1;
    bool node_269_isOutputDirty_OUT : 1;
    bool node_270_isNodeDirty : 1;
    bool node_270_isOutputDirty_OUT : 1;
    bool node_271_isNodeDirty : 1;
    bool node_271_isOutputDirty_OUT : 1;
    bool node_272_isNodeDirty : 1;
    bool node_272_isOutputDirty_OUT : 1;
    bool node_273_isNodeDirty : 1;
    bool node_273_isOutputDirty_OUT : 1;
    bool node_274_isNodeDirty : 1;
    bool node_274_isOutputDirty_OUT : 1;
    bool node_275_isNodeDirty : 1;
    bool node_275_isOutputDirty_OUT : 1;
    bool node_276_isNodeDirty : 1;
    bool node_276_isOutputDirty_OUT : 1;
    bool node_277_isNodeDirty : 1;
    bool node_277_isOutputDirty_OUT : 1;
    bool node_278_isNodeDirty : 1;
    bool node_278_isOutputDirty_OUT : 1;
    bool node_279_isNodeDirty : 1;
    bool node_279_isOutputDirty_OUT : 1;
    bool node_280_isNodeDirty : 1;
    bool node_280_isOutputDirty_OUT : 1;
    bool node_281_isNodeDirty : 1;
    bool node_281_isOutputDirty_OUT : 1;
    bool node_282_isNodeDirty : 1;
    bool node_282_isOutputDirty_OUT : 1;
    bool node_283_isNodeDirty : 1;
    bool node_283_isOutputDirty_OUT : 1;
    bool node_284_isNodeDirty : 1;
    bool node_284_isOutputDirty_OUT : 1;
    bool node_285_isNodeDirty : 1;
    bool node_285_isOutputDirty_OUT : 1;
    bool node_286_isNodeDirty : 1;
    bool node_286_isOutputDirty_OUT : 1;
    bool node_287_isNodeDirty : 1;
    bool node_287_isOutputDirty_OUT : 1;
    bool node_288_isNodeDirty : 1;
    bool node_288_isOutputDirty_OUT : 1;
    bool node_289_isNodeDirty : 1;
    bool node_289_isOutputDirty_OUT : 1;
    bool node_290_isNodeDirty : 1;
    bool node_290_isOutputDirty_OUT : 1;
    bool node_291_isNodeDirty : 1;
    bool node_291_isOutputDirty_OUT : 1;
    bool node_292_isNodeDirty : 1;
    bool node_292_isOutputDirty_OUT : 1;
    bool node_293_isNodeDirty : 1;
    bool node_293_isOutputDirty_OUT : 1;
    bool node_294_isNodeDirty : 1;
    bool node_294_isOutputDirty_OUT : 1;
    bool node_295_isNodeDirty : 1;
    bool node_295_isOutputDirty_OUT : 1;
    bool node_296_isNodeDirty : 1;
    bool node_296_isOutputDirty_OUT : 1;
    bool node_297_isNodeDirty : 1;
    bool node_297_isOutputDirty_OUT : 1;
    bool node_298_isNodeDirty : 1;
    bool node_298_isOutputDirty_OUT : 1;
    bool node_299_isNodeDirty : 1;
    bool node_299_isOutputDirty_OUT : 1;
    bool node_300_isNodeDirty : 1;
    bool node_300_isOutputDirty_OUT : 1;
    bool node_301_isNodeDirty : 1;
    bool node_301_isOutputDirty_OUT : 1;
    bool node_302_isNodeDirty : 1;
    bool node_302_isOutputDirty_OUT : 1;
    bool node_303_isNodeDirty : 1;
    bool node_303_isOutputDirty_OUT : 1;
    bool node_304_isNodeDirty : 1;
    bool node_304_isOutputDirty_OUT : 1;
    bool node_305_isNodeDirty : 1;
    bool node_305_isOutputDirty_OUT : 1;
    bool node_306_isNodeDirty : 1;
    bool node_306_isOutputDirty_OUT : 1;
    bool node_307_isNodeDirty : 1;
    bool node_307_isOutputDirty_OUT : 1;
    bool node_308_isNodeDirty : 1;
    bool node_308_isOutputDirty_OUT : 1;
    bool node_309_isNodeDirty : 1;
    bool node_309_isOutputDirty_OUT : 1;
    bool node_310_isNodeDirty : 1;
    bool node_310_isOutputDirty_OUT : 1;
    bool node_311_isNodeDirty : 1;
    bool node_311_isOutputDirty_OUT : 1;
    bool node_312_isNodeDirty : 1;
    bool node_312_isOutputDirty_OUT : 1;
    bool node_313_isNodeDirty : 1;
    bool node_313_isOutputDirty_OUT : 1;
    bool node_314_isNodeDirty : 1;
    bool node_314_isOutputDirty_OUT : 1;
    bool node_315_isNodeDirty : 1;
    bool node_315_isOutputDirty_OUT : 1;
    bool node_316_isNodeDirty : 1;
    bool node_316_isOutputDirty_OUT : 1;
    bool node_317_isNodeDirty : 1;
    bool node_317_isOutputDirty_OUT : 1;
    bool node_318_isNodeDirty : 1;
    bool node_318_isOutputDirty_OUT : 1;
    bool node_319_isNodeDirty : 1;
    bool node_319_isOutputDirty_OUT : 1;
    bool node_320_isNodeDirty : 1;
    bool node_320_isOutputDirty_OUT : 1;
    bool node_321_isNodeDirty : 1;
    bool node_321_isOutputDirty_OUT : 1;
    bool node_322_isNodeDirty : 1;
    bool node_322_isOutputDirty_OUT : 1;
    bool node_323_isNodeDirty : 1;
    bool node_323_isOutputDirty_OUT : 1;
    bool node_324_isNodeDirty : 1;
    bool node_324_isOutputDirty_OUT : 1;
    bool node_325_isNodeDirty : 1;
    bool node_325_isOutputDirty_OUT : 1;
    bool node_326_isNodeDirty : 1;
    bool node_326_isOutputDirty_OUT : 1;
    bool node_327_isNodeDirty : 1;
    bool node_327_isOutputDirty_OUT : 1;
    bool node_328_isNodeDirty : 1;
    bool node_328_isOutputDirty_OUT : 1;
    bool node_329_isNodeDirty : 1;
    bool node_329_isOutputDirty_OUT : 1;
    bool node_330_isNodeDirty : 1;
    bool node_330_isOutputDirty_OUT : 1;
    bool node_331_isNodeDirty : 1;
    bool node_331_isOutputDirty_OUT : 1;
    bool node_332_isNodeDirty : 1;
    bool node_332_isOutputDirty_OUT : 1;
    bool node_333_isNodeDirty : 1;
    bool node_333_isOutputDirty_OUT : 1;
    bool node_334_isNodeDirty : 1;
    bool node_334_isOutputDirty_OUT : 1;
    bool node_335_isNodeDirty : 1;
    bool node_335_isOutputDirty_OUT : 1;
    bool node_336_isNodeDirty : 1;
    bool node_336_isOutputDirty_OUT : 1;
    bool node_337_isNodeDirty : 1;
    bool node_337_isOutputDirty_OUT : 1;
    bool node_338_isNodeDirty : 1;
    bool node_338_isOutputDirty_OUT : 1;
    bool node_339_isNodeDirty : 1;
    bool node_339_isOutputDirty_OUT : 1;
    bool node_340_isNodeDirty : 1;
    bool node_340_isOutputDirty_OUT : 1;
    bool node_341_isNodeDirty : 1;
    bool node_341_isOutputDirty_OUT : 1;
    bool node_342_isNodeDirty : 1;
    bool node_342_isOutputDirty_DONE : 1;
    bool node_343_isNodeDirty : 1;
    bool node_343_isOutputDirty_OUT : 1;
    bool node_344_isNodeDirty : 1;
    bool node_344_isOutputDirty_OUT : 1;
    bool node_345_isNodeDirty : 1;
    bool node_345_isOutputDirty_DONE : 1;
    bool node_346_isNodeDirty : 1;
    bool node_346_isOutputDirty_OUT : 1;
    bool node_347_isNodeDirty : 1;
    bool node_347_isOutputDirty_OUT : 1;
    bool node_348_isNodeDirty : 1;
    bool node_348_isOutputDirty_DONE : 1;
    bool node_349_isNodeDirty : 1;
    bool node_349_isOutputDirty_OUT : 1;
    bool node_350_isNodeDirty : 1;
    bool node_350_isOutputDirty_OUT : 1;
    bool node_351_isNodeDirty : 1;
    bool node_351_isOutputDirty_DONE : 1;
    bool node_352_isNodeDirty : 1;
    bool node_352_isOutputDirty_OUT : 1;
    bool node_353_isNodeDirty : 1;
    bool node_353_isOutputDirty_OUT : 1;
    bool node_354_isNodeDirty : 1;
    bool node_354_isOutputDirty_DONE : 1;
    bool node_355_isNodeDirty : 1;
    bool node_355_isOutputDirty_OUT : 1;
    bool node_356_isNodeDirty : 1;
    bool node_356_isOutputDirty_OUT : 1;
    bool node_357_isNodeDirty : 1;
    bool node_357_isOutputDirty_DONE : 1;
    bool node_358_isNodeDirty : 1;
    bool node_358_isOutputDirty_OUT : 1;
    bool node_359_isNodeDirty : 1;
    bool node_359_isOutputDirty_OUT : 1;
    bool node_360_isNodeDirty : 1;
    bool node_360_isOutputDirty_DONE : 1;
    bool node_361_isNodeDirty : 1;
    bool node_361_isOutputDirty_OUT : 1;
    bool node_362_isNodeDirty : 1;
    bool node_362_isOutputDirty_OUT : 1;
    bool node_363_isNodeDirty : 1;
    bool node_363_isOutputDirty_DONE : 1;
    bool node_364_isNodeDirty : 1;
    bool node_364_isOutputDirty_OUT : 1;
    bool node_365_isNodeDirty : 1;
    bool node_365_isOutputDirty_OUT : 1;
    bool node_366_isNodeDirty : 1;
    bool node_366_isOutputDirty_DONE : 1;
    bool node_367_isNodeDirty : 1;
    bool node_367_isOutputDirty_OUT : 1;
    bool node_368_isNodeDirty : 1;
    bool node_368_isOutputDirty_OUT : 1;
    bool node_369_isNodeDirty : 1;
    bool node_369_isOutputDirty_DONE : 1;
    bool node_370_isNodeDirty : 1;
    bool node_370_isOutputDirty_OUT : 1;
    bool node_371_isNodeDirty : 1;
    bool node_371_isOutputDirty_OUT : 1;
    bool node_372_isNodeDirty : 1;
    bool node_372_isOutputDirty_DONE : 1;
    bool node_373_isNodeDirty : 1;
    bool node_373_isOutputDirty_OUT : 1;
    bool node_374_isNodeDirty : 1;
    bool node_374_isOutputDirty_DONE : 1;
    bool node_375_isNodeDirty : 1;
    bool node_375_isOutputDirty_OUT : 1;
    bool node_376_isNodeDirty : 1;
    bool node_376_isOutputDirty_DONE : 1;
    bool node_377_isNodeDirty : 1;
    bool node_377_isOutputDirty_OUT : 1;
    bool node_378_isNodeDirty : 1;
    bool node_378_isOutputDirty_DONE : 1;
    bool node_379_isNodeDirty : 1;
    bool node_379_isOutputDirty_OUT : 1;
    bool node_380_isNodeDirty : 1;
    bool node_380_isOutputDirty_DONE : 1;
    bool node_381_isNodeDirty : 1;
    bool node_381_isOutputDirty_OUT : 1;
    bool node_382_isNodeDirty : 1;
    bool node_382_isOutputDirty_DONE : 1;
    bool node_383_isNodeDirty : 1;
    bool node_383_isOutputDirty_OUT : 1;
    bool node_384_isNodeDirty : 1;
    bool node_384_isOutputDirty_DONE : 1;
    bool node_385_isNodeDirty : 1;
    bool node_385_isOutputDirty_OUT : 1;
    bool node_386_isNodeDirty : 1;
    bool node_386_isOutputDirty_DONE : 1;
    bool node_387_isNodeDirty : 1;
    bool node_387_isOutputDirty_OUT : 1;
    bool node_388_isNodeDirty : 1;
    bool node_388_isOutputDirty_DONE : 1;
    bool node_389_isNodeDirty : 1;
    bool node_389_isOutputDirty_OUT : 1;
    bool node_390_isNodeDirty : 1;
    bool node_390_isOutputDirty_DONE : 1;
    bool node_391_isNodeDirty : 1;
    bool node_391_isOutputDirty_OUT : 1;
    bool node_392_isNodeDirty : 1;
    bool node_392_isOutputDirty_DONE : 1;
    bool node_392_hasUpstreamError : 1;
    bool node_393_isNodeDirty : 1;
    bool node_394_isNodeDirty : 1;
    bool node_394_isOutputDirty_DONE : 1;
    bool node_394_hasUpstreamError : 1;
    bool node_395_isNodeDirty : 1;
    bool node_396_isNodeDirty : 1;
    bool node_396_isOutputDirty_DONE : 1;
    bool node_396_hasUpstreamError : 1;
    bool node_397_isNodeDirty : 1;
    bool node_398_isNodeDirty : 1;
    bool node_398_isOutputDirty_DONE : 1;
    bool node_398_hasUpstreamError : 1;
    bool node_399_isNodeDirty : 1;
    bool node_400_isNodeDirty : 1;
    bool node_400_isOutputDirty_DONE : 1;
    bool node_400_hasUpstreamError : 1;
    bool node_401_isNodeDirty : 1;
    bool node_402_isNodeDirty : 1;
    bool node_402_isOutputDirty_DONE : 1;
    bool node_402_hasUpstreamError : 1;
    bool node_403_isNodeDirty : 1;
    bool node_404_isNodeDirty : 1;
    bool node_404_isOutputDirty_DONE : 1;
    bool node_404_hasUpstreamError : 1;
    bool node_405_isNodeDirty : 1;
    bool node_406_isNodeDirty : 1;
    bool node_406_isOutputDirty_DONE : 1;
    bool node_406_hasUpstreamError : 1;
    bool node_407_isNodeDirty : 1;
    bool node_408_isNodeDirty : 1;
    bool node_408_isOutputDirty_DONE : 1;
    bool node_408_hasUpstreamError : 1;
    bool node_409_isNodeDirty : 1;
    bool node_410_isNodeDirty : 1;
    bool node_410_isOutputDirty_DONE : 1;
    bool node_410_hasUpstreamError : 1;
    bool node_411_isNodeDirty : 1;
    bool node_412_isNodeDirty : 1;
    bool node_412_isOutputDirty_DONE : 1;
    bool node_412_hasUpstreamError : 1;
    bool node_413_isNodeDirty : 1;
    bool node_413_isOutputDirty_DONE : 1;
    bool node_413_hasUpstreamError : 1;
    bool node_414_isNodeDirty : 1;
    bool node_414_isOutputDirty_DONE : 1;
    bool node_414_hasUpstreamError : 1;
    bool node_415_isNodeDirty : 1;
    bool node_415_isOutputDirty_DONE : 1;
    bool node_415_hasUpstreamError : 1;
    bool node_416_isNodeDirty : 1;
    bool node_416_isOutputDirty_DONE : 1;
    bool node_416_hasUpstreamError : 1;
    bool node_417_isNodeDirty : 1;
    bool node_417_isOutputDirty_DONE : 1;
    bool node_417_hasUpstreamError : 1;
    bool node_418_isNodeDirty : 1;
    bool node_418_isOutputDirty_DONE : 1;
    bool node_418_hasUpstreamError : 1;
    bool node_419_isNodeDirty : 1;
    bool node_419_isOutputDirty_DONE : 1;
    bool node_419_hasUpstreamError : 1;
    bool node_420_isNodeDirty : 1;
    bool node_420_isOutputDirty_DONE : 1;
    bool node_420_hasUpstreamError : 1;
    bool node_421_isNodeDirty : 1;
    bool node_421_isOutputDirty_DONE : 1;
    bool node_421_hasUpstreamError : 1;
    bool node_422_isNodeDirty : 1;
    bool node_422_isOutputDirty_DONE : 1;
    bool node_422_hasUpstreamError : 1;
    bool node_423_isNodeDirty : 1;
    bool node_423_isOutputDirty_DONE : 1;
    bool node_423_hasUpstreamError : 1;
    bool node_424_isNodeDirty : 1;
    bool node_424_isOutputDirty_DONE : 1;
    bool node_424_hasUpstreamError : 1;
    bool node_425_isNodeDirty : 1;
    bool node_425_isOutputDirty_DONE : 1;
    bool node_425_hasUpstreamError : 1;
    bool node_426_isNodeDirty : 1;
    bool node_426_isOutputDirty_DONE : 1;
    bool node_426_hasUpstreamError : 1;
    bool node_427_isNodeDirty : 1;
    bool node_427_isOutputDirty_DONE : 1;
    bool node_427_hasUpstreamError : 1;
    bool node_428_isNodeDirty : 1;
    bool node_428_isOutputDirty_DONE : 1;
    bool node_428_hasUpstreamError : 1;
    bool node_429_isNodeDirty : 1;
    bool node_429_isOutputDirty_DONE : 1;
    bool node_429_hasUpstreamError : 1;
    bool node_430_isNodeDirty : 1;
    bool node_430_isOutputDirty_DONE : 1;
    bool node_430_hasUpstreamError : 1;
    bool node_431_isNodeDirty : 1;
    bool node_431_isOutputDirty_DONE : 1;
    bool node_431_hasUpstreamError : 1;
    bool node_432_isNodeDirty : 1;
    bool node_432_isOutputDirty_DONE : 1;
    bool node_432_hasUpstreamError : 1;
    bool node_433_isNodeDirty : 1;
    bool node_433_isOutputDirty_DONE : 1;
    bool node_433_hasUpstreamError : 1;
    bool node_434_isNodeDirty : 1;
    bool node_434_isOutputDirty_DONE : 1;
    bool node_434_hasUpstreamError : 1;
    bool node_435_isNodeDirty : 1;
    bool node_435_isOutputDirty_DONE : 1;
    bool node_435_hasUpstreamError : 1;
    bool node_436_isNodeDirty : 1;
    bool node_436_isOutputDirty_DONE : 1;
    bool node_436_hasUpstreamError : 1;
    bool node_437_isNodeDirty : 1;
    bool node_437_isOutputDirty_DONE : 1;
    bool node_437_hasUpstreamError : 1;
    bool node_438_isNodeDirty : 1;
    bool node_438_isOutputDirty_DONE : 1;
    bool node_438_hasUpstreamError : 1;
    bool node_439_isNodeDirty : 1;
    bool node_439_isOutputDirty_DONE : 1;
    bool node_439_hasUpstreamError : 1;
    bool node_440_isNodeDirty : 1;
    bool node_440_isOutputDirty_DONE : 1;
    bool node_440_hasUpstreamError : 1;
    bool node_441_isNodeDirty : 1;
    bool node_441_isOutputDirty_DONE : 1;
    bool node_441_hasUpstreamError : 1;
    bool node_442_isNodeDirty : 1;
    bool node_442_isOutputDirty_DONE : 1;
    bool node_442_hasUpstreamError : 1;
    bool node_443_isNodeDirty : 1;
    bool node_443_isOutputDirty_DONE : 1;
    bool node_443_hasUpstreamError : 1;
    bool node_444_isNodeDirty : 1;
    bool node_444_isOutputDirty_DONE : 1;
    bool node_444_hasUpstreamError : 1;
    bool node_445_isNodeDirty : 1;
    bool node_445_isOutputDirty_DONE : 1;
    bool node_445_hasUpstreamError : 1;
    bool node_446_isNodeDirty : 1;
    bool node_446_isOutputDirty_DONE : 1;
    bool node_446_hasUpstreamError : 1;
    bool node_447_isNodeDirty : 1;
    bool node_447_isOutputDirty_DONE : 1;
    bool node_447_hasUpstreamError : 1;
    bool node_448_isNodeDirty : 1;
    bool node_448_isOutputDirty_DONE : 1;
    bool node_448_hasUpstreamError : 1;
    bool node_449_isNodeDirty : 1;
    bool node_449_isOutputDirty_DONE : 1;
    bool node_449_hasUpstreamError : 1;
    bool node_450_isNodeDirty : 1;
    bool node_450_isOutputDirty_DONE : 1;
    bool node_450_hasUpstreamError : 1;
    bool node_451_isNodeDirty : 1;
    bool node_451_isOutputDirty_DONE : 1;
    bool node_451_hasUpstreamError : 1;
    bool node_452_isNodeDirty : 1;
    bool node_452_isOutputDirty_DONE : 1;
    bool node_452_hasUpstreamError : 1;
    bool node_453_isNodeDirty : 1;
    bool node_453_isOutputDirty_DONE : 1;
    bool node_453_hasUpstreamError : 1;
    bool node_454_isNodeDirty : 1;
    bool node_454_isOutputDirty_DONE : 1;
    bool node_454_hasUpstreamError : 1;
    bool node_455_isNodeDirty : 1;
    bool node_455_isOutputDirty_DONE : 1;
    bool node_455_hasUpstreamError : 1;
    bool node_456_isNodeDirty : 1;
    bool node_456_isOutputDirty_DONE : 1;
    bool node_456_hasUpstreamError : 1;
    bool node_457_isNodeDirty : 1;
    bool node_457_isOutputDirty_DONE : 1;
    bool node_457_hasUpstreamError : 1;
    bool node_458_isNodeDirty : 1;
    bool node_458_isOutputDirty_DONE : 1;
    bool node_458_hasUpstreamError : 1;
    bool node_459_isNodeDirty : 1;
    bool node_459_isOutputDirty_DONE : 1;
    bool node_459_hasUpstreamError : 1;
    bool node_460_isNodeDirty : 1;
    bool node_460_isOutputDirty_DONE : 1;
    bool node_460_hasUpstreamError : 1;
    bool node_461_isNodeDirty : 1;
    bool node_461_isOutputDirty_DONE : 1;
    bool node_461_hasUpstreamError : 1;
    bool node_462_isNodeDirty : 1;
    bool node_462_isOutputDirty_DONE : 1;
    bool node_462_hasUpstreamError : 1;
    bool node_463_isNodeDirty : 1;
    bool node_463_isOutputDirty_DONE : 1;
    bool node_463_hasUpstreamError : 1;
    bool node_464_isNodeDirty : 1;
    bool node_464_isOutputDirty_DONE : 1;
    bool node_464_hasUpstreamError : 1;
    bool node_465_isNodeDirty : 1;
    bool node_465_isOutputDirty_DONE : 1;
    bool node_465_hasUpstreamError : 1;
    bool node_466_isNodeDirty : 1;
    bool node_466_isOutputDirty_DONE : 1;
    bool node_466_hasUpstreamError : 1;
    bool node_467_isNodeDirty : 1;
    bool node_467_isOutputDirty_DONE : 1;
    bool node_467_hasUpstreamError : 1;
    bool node_468_isNodeDirty : 1;
    bool node_468_isOutputDirty_DONE : 1;
    bool node_468_hasUpstreamError : 1;
    bool node_469_isNodeDirty : 1;
    bool node_469_isOutputDirty_DONE : 1;
    bool node_469_hasUpstreamError : 1;
    bool node_470_isNodeDirty : 1;
    bool node_470_isOutputDirty_DONE : 1;
    bool node_470_hasUpstreamError : 1;
    bool node_471_isNodeDirty : 1;
    bool node_471_isOutputDirty_DONE : 1;
    bool node_471_hasUpstreamError : 1;
    bool node_472_isNodeDirty : 1;
    bool node_472_isOutputDirty_DONE : 1;
    bool node_472_hasUpstreamError : 1;
    bool node_473_isNodeDirty : 1;
    bool node_473_isOutputDirty_DONE : 1;
    bool node_473_hasUpstreamError : 1;
    bool node_474_isNodeDirty : 1;
    bool node_474_isOutputDirty_DONE : 1;
    bool node_474_hasUpstreamError : 1;
    bool node_475_isNodeDirty : 1;
    bool node_475_isOutputDirty_DONE : 1;
    bool node_475_hasUpstreamError : 1;
    bool node_476_isNodeDirty : 1;
    bool node_476_isOutputDirty_DONE : 1;
    bool node_476_hasUpstreamError : 1;
    bool node_477_isNodeDirty : 1;
    bool node_477_isOutputDirty_DONE : 1;
    bool node_477_hasUpstreamError : 1;
    bool node_478_isNodeDirty : 1;
    bool node_478_isOutputDirty_DONE : 1;
    bool node_478_hasUpstreamError : 1;
    bool node_479_isNodeDirty : 1;
    bool node_479_isOutputDirty_DONE : 1;
    bool node_479_hasUpstreamError : 1;
    bool node_480_isNodeDirty : 1;
    bool node_480_isOutputDirty_DONE : 1;
    bool node_480_hasUpstreamError : 1;
    bool node_481_isNodeDirty : 1;
    bool node_481_isOutputDirty_DONE : 1;
    bool node_481_hasUpstreamError : 1;
    bool node_482_isNodeDirty : 1;
    bool node_482_hasUpstreamError : 1;
    bool node_483_isNodeDirty : 1;
    bool node_483_hasUpstreamError : 1;
    bool node_484_isNodeDirty : 1;
    bool node_484_hasUpstreamError : 1;
    bool node_485_isNodeDirty : 1;
    bool node_485_hasUpstreamError : 1;
    bool node_486_isNodeDirty : 1;
    bool node_486_hasUpstreamError : 1;
    bool node_487_isNodeDirty : 1;
    bool node_487_hasUpstreamError : 1;
    bool node_488_isNodeDirty : 1;
    bool node_488_hasUpstreamError : 1;
    bool node_489_isNodeDirty : 1;
    bool node_489_hasUpstreamError : 1;
    bool node_490_isNodeDirty : 1;
    bool node_490_hasUpstreamError : 1;
    bool node_491_isNodeDirty : 1;
    bool node_491_hasUpstreamError : 1;
    TransactionState() {
        node_0_isNodeDirty = true;
        node_0_isOutputDirty_OUT = true;
        node_1_isNodeDirty = true;
        node_1_isOutputDirty_OUT = true;
        node_2_isNodeDirty = true;
        node_2_isOutputDirty_OUT = true;
        node_3_isNodeDirty = true;
        node_3_isOutputDirty_OUT = true;
        node_4_isNodeDirty = true;
        node_4_isOutputDirty_OUT = true;
        node_5_isNodeDirty = true;
        node_5_isOutputDirty_OUT = true;
        node_6_isNodeDirty = true;
        node_6_isOutputDirty_OUT = true;
        node_7_isNodeDirty = true;
        node_7_isOutputDirty_OUT = true;
        node_8_isNodeDirty = true;
        node_8_isOutputDirty_OUT = true;
        node_9_isNodeDirty = true;
        node_9_isOutputDirty_OUT = true;
        node_190_isNodeDirty = true;
        node_190_isOutputDirty_TICK = false;
        node_191_isNodeDirty = true;
        node_191_isOutputDirty_BOOT = false;
        node_192_isNodeDirty = true;
        node_192_isOutputDirty_DEV = true;
        node_193_isNodeDirty = true;
        node_193_isOutputDirty_OUT = true;
        node_194_isNodeDirty = true;
        node_194_isOutputDirty_OUT = false;
        node_195_isNodeDirty = true;
        node_195_isOutputDirty_DEV = true;
        node_196_isNodeDirty = true;
        node_196_isOutputDirty_OUT = true;
        node_197_isNodeDirty = true;
        node_197_isOutputDirty_OUT = false;
        node_198_isNodeDirty = true;
        node_198_isOutputDirty_DEV = true;
        node_199_isNodeDirty = true;
        node_199_isOutputDirty_OUT = true;
        node_200_isNodeDirty = true;
        node_200_isOutputDirty_OUT = false;
        node_201_isNodeDirty = true;
        node_201_isOutputDirty_DEV = true;
        node_202_isNodeDirty = true;
        node_202_isOutputDirty_OUT = true;
        node_203_isNodeDirty = true;
        node_203_isOutputDirty_OUT = false;
        node_204_isNodeDirty = true;
        node_204_isOutputDirty_DEV = true;
        node_205_isNodeDirty = true;
        node_205_isOutputDirty_OUT = true;
        node_206_isNodeDirty = true;
        node_206_isOutputDirty_OUT = false;
        node_207_isNodeDirty = true;
        node_207_isOutputDirty_DEV = true;
        node_208_isNodeDirty = true;
        node_208_isOutputDirty_OUT = true;
        node_209_isNodeDirty = true;
        node_209_isOutputDirty_OUT = false;
        node_210_isNodeDirty = true;
        node_210_isOutputDirty_DEV = true;
        node_211_isNodeDirty = true;
        node_211_isOutputDirty_OUT = true;
        node_212_isNodeDirty = true;
        node_212_isOutputDirty_OUT = false;
        node_213_isNodeDirty = true;
        node_213_isOutputDirty_DEV = true;
        node_214_isNodeDirty = true;
        node_214_isOutputDirty_OUT = true;
        node_215_isNodeDirty = true;
        node_215_isOutputDirty_OUT = false;
        node_216_isNodeDirty = true;
        node_216_isOutputDirty_DEV = true;
        node_217_isNodeDirty = true;
        node_217_isOutputDirty_OUT = true;
        node_218_isNodeDirty = true;
        node_218_isOutputDirty_OUT = false;
        node_219_isNodeDirty = true;
        node_219_isOutputDirty_DEV = true;
        node_220_isNodeDirty = true;
        node_220_isOutputDirty_OUT = true;
        node_221_isNodeDirty = true;
        node_221_isOutputDirty_OUT = false;
        node_222_isNodeDirty = true;
        node_222_isOutputDirty_SIG = true;
        node_223_isNodeDirty = true;
        node_223_isOutputDirty_SIG = true;
        node_224_isNodeDirty = true;
        node_224_isOutputDirty_SIG = true;
        node_225_isNodeDirty = true;
        node_225_isOutputDirty_SIG = true;
        node_226_isNodeDirty = true;
        node_226_isOutputDirty_SIG = true;
        node_227_isNodeDirty = true;
        node_227_isOutputDirty_SIG = true;
        node_228_isNodeDirty = true;
        node_228_isOutputDirty_SIG = true;
        node_229_isNodeDirty = true;
        node_229_isOutputDirty_SIG = true;
        node_230_isNodeDirty = true;
        node_230_isOutputDirty_SIG = true;
        node_231_isNodeDirty = true;
        node_231_isOutputDirty_SIG = true;
        node_232_isNodeDirty = true;
        node_232_isOutputDirty_OUT = false;
        node_233_isNodeDirty = true;
        node_233_isOutputDirty_OUT = false;
        node_234_isNodeDirty = true;
        node_234_isOutputDirty_OUT = false;
        node_235_isNodeDirty = true;
        node_235_isOutputDirty_OUT = false;
        node_236_isNodeDirty = true;
        node_236_isOutputDirty_OUT = false;
        node_237_isNodeDirty = true;
        node_237_isOutputDirty_OUT = false;
        node_238_isNodeDirty = true;
        node_238_isOutputDirty_OUT = false;
        node_239_isNodeDirty = true;
        node_239_isOutputDirty_OUT = false;
        node_240_isNodeDirty = true;
        node_240_isOutputDirty_OUT = false;
        node_241_isNodeDirty = true;
        node_241_isOutputDirty_OUT = false;
        node_242_isNodeDirty = true;
        node_242_isOutputDirty_OUT = false;
        node_243_isNodeDirty = true;
        node_243_isOutputDirty_OUT = false;
        node_244_isNodeDirty = true;
        node_244_isOutputDirty_OUT = false;
        node_245_isNodeDirty = true;
        node_245_isOutputDirty_OUT = false;
        node_246_isNodeDirty = true;
        node_246_isOutputDirty_OUT = false;
        node_247_isNodeDirty = true;
        node_247_isOutputDirty_OUT = false;
        node_248_isNodeDirty = true;
        node_248_isOutputDirty_OUT = false;
        node_249_isNodeDirty = true;
        node_249_isOutputDirty_OUT = false;
        node_250_isNodeDirty = true;
        node_250_isOutputDirty_OUT = false;
        node_251_isNodeDirty = true;
        node_251_isOutputDirty_OUT = false;
        node_252_isNodeDirty = true;
        node_253_isNodeDirty = true;
        node_254_isNodeDirty = true;
        node_255_isNodeDirty = true;
        node_256_isNodeDirty = true;
        node_257_isNodeDirty = true;
        node_258_isNodeDirty = true;
        node_259_isNodeDirty = true;
        node_260_isNodeDirty = true;
        node_261_isNodeDirty = true;
        node_262_isNodeDirty = true;
        node_262_isOutputDirty_OUT = false;
        node_263_isNodeDirty = true;
        node_263_isOutputDirty_OUT = false;
        node_264_isNodeDirty = true;
        node_264_isOutputDirty_OUT = false;
        node_265_isNodeDirty = true;
        node_265_isOutputDirty_OUT = false;
        node_266_isNodeDirty = true;
        node_266_isOutputDirty_OUT = false;
        node_267_isNodeDirty = true;
        node_267_isOutputDirty_OUT = false;
        node_268_isNodeDirty = true;
        node_268_isOutputDirty_OUT = false;
        node_269_isNodeDirty = true;
        node_269_isOutputDirty_OUT = false;
        node_270_isNodeDirty = true;
        node_270_isOutputDirty_OUT = false;
        node_271_isNodeDirty = true;
        node_271_isOutputDirty_OUT = false;
        node_272_isNodeDirty = true;
        node_272_isOutputDirty_OUT = true;
        node_273_isNodeDirty = true;
        node_273_isOutputDirty_OUT = true;
        node_274_isNodeDirty = true;
        node_274_isOutputDirty_OUT = true;
        node_275_isNodeDirty = true;
        node_275_isOutputDirty_OUT = true;
        node_276_isNodeDirty = true;
        node_276_isOutputDirty_OUT = true;
        node_277_isNodeDirty = true;
        node_277_isOutputDirty_OUT = true;
        node_278_isNodeDirty = true;
        node_278_isOutputDirty_OUT = true;
        node_279_isNodeDirty = true;
        node_279_isOutputDirty_OUT = true;
        node_280_isNodeDirty = true;
        node_280_isOutputDirty_OUT = true;
        node_281_isNodeDirty = true;
        node_281_isOutputDirty_OUT = true;
        node_282_isNodeDirty = true;
        node_283_isNodeDirty = true;
        node_283_isOutputDirty_OUT = false;
        node_284_isNodeDirty = true;
        node_284_isOutputDirty_OUT = false;
        node_285_isNodeDirty = true;
        node_286_isNodeDirty = true;
        node_286_isOutputDirty_OUT = false;
        node_287_isNodeDirty = true;
        node_287_isOutputDirty_OUT = false;
        node_288_isNodeDirty = true;
        node_289_isNodeDirty = true;
        node_289_isOutputDirty_OUT = false;
        node_290_isNodeDirty = true;
        node_290_isOutputDirty_OUT = false;
        node_291_isNodeDirty = true;
        node_292_isNodeDirty = true;
        node_292_isOutputDirty_OUT = false;
        node_293_isNodeDirty = true;
        node_293_isOutputDirty_OUT = false;
        node_294_isNodeDirty = true;
        node_295_isNodeDirty = true;
        node_295_isOutputDirty_OUT = false;
        node_296_isNodeDirty = true;
        node_296_isOutputDirty_OUT = false;
        node_297_isNodeDirty = true;
        node_298_isNodeDirty = true;
        node_298_isOutputDirty_OUT = false;
        node_299_isNodeDirty = true;
        node_299_isOutputDirty_OUT = false;
        node_300_isNodeDirty = true;
        node_301_isNodeDirty = true;
        node_301_isOutputDirty_OUT = false;
        node_302_isNodeDirty = true;
        node_302_isOutputDirty_OUT = false;
        node_303_isNodeDirty = true;
        node_304_isNodeDirty = true;
        node_304_isOutputDirty_OUT = false;
        node_305_isNodeDirty = true;
        node_305_isOutputDirty_OUT = false;
        node_306_isNodeDirty = true;
        node_307_isNodeDirty = true;
        node_307_isOutputDirty_OUT = false;
        node_308_isNodeDirty = true;
        node_308_isOutputDirty_OUT = false;
        node_309_isNodeDirty = true;
        node_310_isNodeDirty = true;
        node_310_isOutputDirty_OUT = false;
        node_311_isNodeDirty = true;
        node_311_isOutputDirty_OUT = false;
        node_312_isNodeDirty = true;
        node_312_isOutputDirty_OUT = false;
        node_313_isNodeDirty = true;
        node_313_isOutputDirty_OUT = false;
        node_314_isNodeDirty = true;
        node_314_isOutputDirty_OUT = false;
        node_315_isNodeDirty = true;
        node_315_isOutputDirty_OUT = false;
        node_316_isNodeDirty = true;
        node_316_isOutputDirty_OUT = false;
        node_317_isNodeDirty = true;
        node_317_isOutputDirty_OUT = false;
        node_318_isNodeDirty = true;
        node_318_isOutputDirty_OUT = false;
        node_319_isNodeDirty = true;
        node_319_isOutputDirty_OUT = false;
        node_320_isNodeDirty = true;
        node_320_isOutputDirty_OUT = false;
        node_321_isNodeDirty = true;
        node_321_isOutputDirty_OUT = false;
        node_322_isNodeDirty = true;
        node_322_isOutputDirty_OUT = false;
        node_323_isNodeDirty = true;
        node_323_isOutputDirty_OUT = false;
        node_324_isNodeDirty = true;
        node_324_isOutputDirty_OUT = false;
        node_325_isNodeDirty = true;
        node_325_isOutputDirty_OUT = false;
        node_326_isNodeDirty = true;
        node_326_isOutputDirty_OUT = false;
        node_327_isNodeDirty = true;
        node_327_isOutputDirty_OUT = false;
        node_328_isNodeDirty = true;
        node_328_isOutputDirty_OUT = false;
        node_329_isNodeDirty = true;
        node_329_isOutputDirty_OUT = false;
        node_330_isNodeDirty = true;
        node_330_isOutputDirty_OUT = false;
        node_331_isNodeDirty = true;
        node_331_isOutputDirty_OUT = false;
        node_332_isNodeDirty = true;
        node_332_isOutputDirty_OUT = false;
        node_333_isNodeDirty = true;
        node_333_isOutputDirty_OUT = false;
        node_334_isNodeDirty = true;
        node_334_isOutputDirty_OUT = false;
        node_335_isNodeDirty = true;
        node_335_isOutputDirty_OUT = false;
        node_336_isNodeDirty = true;
        node_336_isOutputDirty_OUT = false;
        node_337_isNodeDirty = true;
        node_337_isOutputDirty_OUT = false;
        node_338_isNodeDirty = true;
        node_338_isOutputDirty_OUT = false;
        node_339_isNodeDirty = true;
        node_339_isOutputDirty_OUT = false;
        node_340_isNodeDirty = true;
        node_340_isOutputDirty_OUT = false;
        node_341_isNodeDirty = true;
        node_341_isOutputDirty_OUT = false;
        node_342_isNodeDirty = true;
        node_342_isOutputDirty_DONE = false;
        node_343_isNodeDirty = true;
        node_343_isOutputDirty_OUT = false;
        node_344_isNodeDirty = true;
        node_344_isOutputDirty_OUT = false;
        node_345_isNodeDirty = true;
        node_345_isOutputDirty_DONE = false;
        node_346_isNodeDirty = true;
        node_346_isOutputDirty_OUT = false;
        node_347_isNodeDirty = true;
        node_347_isOutputDirty_OUT = false;
        node_348_isNodeDirty = true;
        node_348_isOutputDirty_DONE = false;
        node_349_isNodeDirty = true;
        node_349_isOutputDirty_OUT = false;
        node_350_isNodeDirty = true;
        node_350_isOutputDirty_OUT = false;
        node_351_isNodeDirty = true;
        node_351_isOutputDirty_DONE = false;
        node_352_isNodeDirty = true;
        node_352_isOutputDirty_OUT = false;
        node_353_isNodeDirty = true;
        node_353_isOutputDirty_OUT = false;
        node_354_isNodeDirty = true;
        node_354_isOutputDirty_DONE = false;
        node_355_isNodeDirty = true;
        node_355_isOutputDirty_OUT = false;
        node_356_isNodeDirty = true;
        node_356_isOutputDirty_OUT = false;
        node_357_isNodeDirty = true;
        node_357_isOutputDirty_DONE = false;
        node_358_isNodeDirty = true;
        node_358_isOutputDirty_OUT = false;
        node_359_isNodeDirty = true;
        node_359_isOutputDirty_OUT = false;
        node_360_isNodeDirty = true;
        node_360_isOutputDirty_DONE = false;
        node_361_isNodeDirty = true;
        node_361_isOutputDirty_OUT = false;
        node_362_isNodeDirty = true;
        node_362_isOutputDirty_OUT = false;
        node_363_isNodeDirty = true;
        node_363_isOutputDirty_DONE = false;
        node_364_isNodeDirty = true;
        node_364_isOutputDirty_OUT = false;
        node_365_isNodeDirty = true;
        node_365_isOutputDirty_OUT = false;
        node_366_isNodeDirty = true;
        node_366_isOutputDirty_DONE = false;
        node_367_isNodeDirty = true;
        node_367_isOutputDirty_OUT = false;
        node_368_isNodeDirty = true;
        node_368_isOutputDirty_OUT = false;
        node_369_isNodeDirty = true;
        node_369_isOutputDirty_DONE = false;
        node_370_isNodeDirty = true;
        node_370_isOutputDirty_OUT = false;
        node_371_isNodeDirty = true;
        node_371_isOutputDirty_OUT = false;
        node_372_isNodeDirty = true;
        node_372_isOutputDirty_DONE = false;
        node_373_isNodeDirty = true;
        node_373_isOutputDirty_OUT = false;
        node_374_isNodeDirty = true;
        node_374_isOutputDirty_DONE = false;
        node_375_isNodeDirty = true;
        node_375_isOutputDirty_OUT = false;
        node_376_isNodeDirty = true;
        node_376_isOutputDirty_DONE = false;
        node_377_isNodeDirty = true;
        node_377_isOutputDirty_OUT = false;
        node_378_isNodeDirty = true;
        node_378_isOutputDirty_DONE = false;
        node_379_isNodeDirty = true;
        node_379_isOutputDirty_OUT = false;
        node_380_isNodeDirty = true;
        node_380_isOutputDirty_DONE = false;
        node_381_isNodeDirty = true;
        node_381_isOutputDirty_OUT = false;
        node_382_isNodeDirty = true;
        node_382_isOutputDirty_DONE = false;
        node_383_isNodeDirty = true;
        node_383_isOutputDirty_OUT = false;
        node_384_isNodeDirty = true;
        node_384_isOutputDirty_DONE = false;
        node_385_isNodeDirty = true;
        node_385_isOutputDirty_OUT = false;
        node_386_isNodeDirty = true;
        node_386_isOutputDirty_DONE = false;
        node_387_isNodeDirty = true;
        node_387_isOutputDirty_OUT = false;
        node_388_isNodeDirty = true;
        node_388_isOutputDirty_DONE = false;
        node_389_isNodeDirty = true;
        node_389_isOutputDirty_OUT = false;
        node_390_isNodeDirty = true;
        node_390_isOutputDirty_DONE = false;
        node_391_isNodeDirty = true;
        node_391_isOutputDirty_OUT = false;
        node_392_isNodeDirty = true;
        node_392_isOutputDirty_DONE = false;
        node_393_isNodeDirty = true;
        node_394_isNodeDirty = true;
        node_394_isOutputDirty_DONE = false;
        node_395_isNodeDirty = true;
        node_396_isNodeDirty = true;
        node_396_isOutputDirty_DONE = false;
        node_397_isNodeDirty = true;
        node_398_isNodeDirty = true;
        node_398_isOutputDirty_DONE = false;
        node_399_isNodeDirty = true;
        node_400_isNodeDirty = true;
        node_400_isOutputDirty_DONE = false;
        node_401_isNodeDirty = true;
        node_402_isNodeDirty = true;
        node_402_isOutputDirty_DONE = false;
        node_403_isNodeDirty = true;
        node_404_isNodeDirty = true;
        node_404_isOutputDirty_DONE = false;
        node_405_isNodeDirty = true;
        node_406_isNodeDirty = true;
        node_406_isOutputDirty_DONE = false;
        node_407_isNodeDirty = true;
        node_408_isNodeDirty = true;
        node_408_isOutputDirty_DONE = false;
        node_409_isNodeDirty = true;
        node_410_isNodeDirty = true;
        node_410_isOutputDirty_DONE = false;
        node_411_isNodeDirty = true;
        node_412_isNodeDirty = true;
        node_412_isOutputDirty_DONE = false;
        node_413_isNodeDirty = true;
        node_413_isOutputDirty_DONE = false;
        node_414_isNodeDirty = true;
        node_414_isOutputDirty_DONE = false;
        node_415_isNodeDirty = true;
        node_415_isOutputDirty_DONE = false;
        node_416_isNodeDirty = true;
        node_416_isOutputDirty_DONE = false;
        node_417_isNodeDirty = true;
        node_417_isOutputDirty_DONE = false;
        node_418_isNodeDirty = true;
        node_418_isOutputDirty_DONE = false;
        node_419_isNodeDirty = true;
        node_419_isOutputDirty_DONE = false;
        node_420_isNodeDirty = true;
        node_420_isOutputDirty_DONE = false;
        node_421_isNodeDirty = true;
        node_421_isOutputDirty_DONE = false;
        node_422_isNodeDirty = true;
        node_422_isOutputDirty_DONE = false;
        node_423_isNodeDirty = true;
        node_423_isOutputDirty_DONE = false;
        node_424_isNodeDirty = true;
        node_424_isOutputDirty_DONE = false;
        node_425_isNodeDirty = true;
        node_425_isOutputDirty_DONE = false;
        node_426_isNodeDirty = true;
        node_426_isOutputDirty_DONE = false;
        node_427_isNodeDirty = true;
        node_427_isOutputDirty_DONE = false;
        node_428_isNodeDirty = true;
        node_428_isOutputDirty_DONE = false;
        node_429_isNodeDirty = true;
        node_429_isOutputDirty_DONE = false;
        node_430_isNodeDirty = true;
        node_430_isOutputDirty_DONE = false;
        node_431_isNodeDirty = true;
        node_431_isOutputDirty_DONE = false;
        node_432_isNodeDirty = true;
        node_432_isOutputDirty_DONE = false;
        node_433_isNodeDirty = true;
        node_433_isOutputDirty_DONE = false;
        node_434_isNodeDirty = true;
        node_434_isOutputDirty_DONE = false;
        node_435_isNodeDirty = true;
        node_435_isOutputDirty_DONE = false;
        node_436_isNodeDirty = true;
        node_436_isOutputDirty_DONE = false;
        node_437_isNodeDirty = true;
        node_437_isOutputDirty_DONE = false;
        node_438_isNodeDirty = true;
        node_438_isOutputDirty_DONE = false;
        node_439_isNodeDirty = true;
        node_439_isOutputDirty_DONE = false;
        node_440_isNodeDirty = true;
        node_440_isOutputDirty_DONE = false;
        node_441_isNodeDirty = true;
        node_441_isOutputDirty_DONE = false;
        node_442_isNodeDirty = true;
        node_442_isOutputDirty_DONE = false;
        node_443_isNodeDirty = true;
        node_443_isOutputDirty_DONE = false;
        node_444_isNodeDirty = true;
        node_444_isOutputDirty_DONE = false;
        node_445_isNodeDirty = true;
        node_445_isOutputDirty_DONE = false;
        node_446_isNodeDirty = true;
        node_446_isOutputDirty_DONE = false;
        node_447_isNodeDirty = true;
        node_447_isOutputDirty_DONE = false;
        node_448_isNodeDirty = true;
        node_448_isOutputDirty_DONE = false;
        node_449_isNodeDirty = true;
        node_449_isOutputDirty_DONE = false;
        node_450_isNodeDirty = true;
        node_450_isOutputDirty_DONE = false;
        node_451_isNodeDirty = true;
        node_451_isOutputDirty_DONE = false;
        node_452_isNodeDirty = true;
        node_452_isOutputDirty_DONE = false;
        node_453_isNodeDirty = true;
        node_453_isOutputDirty_DONE = false;
        node_454_isNodeDirty = true;
        node_454_isOutputDirty_DONE = false;
        node_455_isNodeDirty = true;
        node_455_isOutputDirty_DONE = false;
        node_456_isNodeDirty = true;
        node_456_isOutputDirty_DONE = false;
        node_457_isNodeDirty = true;
        node_457_isOutputDirty_DONE = false;
        node_458_isNodeDirty = true;
        node_458_isOutputDirty_DONE = false;
        node_459_isNodeDirty = true;
        node_459_isOutputDirty_DONE = false;
        node_460_isNodeDirty = true;
        node_460_isOutputDirty_DONE = false;
        node_461_isNodeDirty = true;
        node_461_isOutputDirty_DONE = false;
        node_462_isNodeDirty = true;
        node_462_isOutputDirty_DONE = false;
        node_463_isNodeDirty = true;
        node_463_isOutputDirty_DONE = false;
        node_464_isNodeDirty = true;
        node_464_isOutputDirty_DONE = false;
        node_465_isNodeDirty = true;
        node_465_isOutputDirty_DONE = false;
        node_466_isNodeDirty = true;
        node_466_isOutputDirty_DONE = false;
        node_467_isNodeDirty = true;
        node_467_isOutputDirty_DONE = false;
        node_468_isNodeDirty = true;
        node_468_isOutputDirty_DONE = false;
        node_469_isNodeDirty = true;
        node_469_isOutputDirty_DONE = false;
        node_470_isNodeDirty = true;
        node_470_isOutputDirty_DONE = false;
        node_471_isNodeDirty = true;
        node_471_isOutputDirty_DONE = false;
        node_472_isNodeDirty = true;
        node_472_isOutputDirty_DONE = false;
        node_473_isNodeDirty = true;
        node_473_isOutputDirty_DONE = false;
        node_474_isNodeDirty = true;
        node_474_isOutputDirty_DONE = false;
        node_475_isNodeDirty = true;
        node_475_isOutputDirty_DONE = false;
        node_476_isNodeDirty = true;
        node_476_isOutputDirty_DONE = false;
        node_477_isNodeDirty = true;
        node_477_isOutputDirty_DONE = false;
        node_478_isNodeDirty = true;
        node_478_isOutputDirty_DONE = false;
        node_479_isNodeDirty = true;
        node_479_isOutputDirty_DONE = false;
        node_480_isNodeDirty = true;
        node_480_isOutputDirty_DONE = false;
        node_481_isNodeDirty = true;
        node_481_isOutputDirty_DONE = false;
        node_482_isNodeDirty = true;
        node_483_isNodeDirty = true;
        node_484_isNodeDirty = true;
        node_485_isNodeDirty = true;
        node_486_isNodeDirty = true;
        node_487_isNodeDirty = true;
        node_488_isNodeDirty = true;
        node_489_isNodeDirty = true;
        node_490_isNodeDirty = true;
        node_491_isNodeDirty = true;
    }
};

TransactionState g_transaction;

typedef xod__debug__tweak_color::Node Node_0;
Node_0 node_0 = Node_0(/* RGB */ { 0xFF, 0x0D, 0x00 });

typedef xod__debug__tweak_color::Node Node_1;
Node_1 node_1 = Node_1(/* RGB */ { 0xC5, 0x8F, 0x07 });

typedef xod__debug__tweak_color::Node Node_2;
Node_2 node_2 = Node_2(/* RGB */ { 0x08, 0x00, 0xFF });

typedef xod__debug__tweak_color::Node Node_3;
Node_3 node_3 = Node_3(/* RGB */ { 0x17, 0xAD, 0x00 });

typedef xod__debug__tweak_color::Node Node_4;
Node_4 node_4 = Node_4(/* RGB */ { 0x00, 0x32, 0x9E });

typedef xod__debug__tweak_color::Node Node_5;
Node_5 node_5 = Node_5(/* RGB */ { 0x08, 0xFF, 0x00 });

typedef xod__debug__tweak_color::Node Node_6;
Node_6 node_6 = Node_6(/* RGB */ { 0x9E, 0x10, 0x00 });

typedef xod__debug__tweak_color::Node Node_7;
Node_7 node_7 = Node_7(/* RGB */ { 0xC7, 0x67, 0x00 });

typedef xod__debug__tweak_color::Node Node_8;
Node_8 node_8 = Node_8(/* RGB */ { 0xC7, 0x81, 0x00 });

typedef xod__debug__tweak_color::Node Node_9;
Node_9 node_9 = Node_9(/* RGB */ { 0xFF, 0x2F, 0x00 });

typedef xod__core__continuously::Node Node_190;
Node_190 node_190 = Node_190();

typedef xod__core__boot::Node Node_191;
Node_191 node_191 = Node_191();

typedef xod_dev__ws2812__ws2812_device::Node<node_88_output_VAL> Node_192;
Node_192 node_192 = Node_192({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_193;
Node_193 node_193 = Node_193(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_194;
Node_194 node_194 = Node_194();

typedef xod_dev__ws2812__ws2812_device::Node<node_93_output_VAL> Node_195;
Node_195 node_195 = Node_195({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_196;
Node_196 node_196 = Node_196(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_197;
Node_197 node_197 = Node_197();

typedef xod_dev__ws2812__ws2812_device::Node<node_98_output_VAL> Node_198;
Node_198 node_198 = Node_198({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_199;
Node_199 node_199 = Node_199(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_200;
Node_200 node_200 = Node_200();

typedef xod_dev__ws2812__ws2812_device::Node<node_103_output_VAL> Node_201;
Node_201 node_201 = Node_201({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_202;
Node_202 node_202 = Node_202(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_203;
Node_203 node_203 = Node_203();

typedef xod_dev__ws2812__ws2812_device::Node<node_108_output_VAL> Node_204;
Node_204 node_204 = Node_204({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_205;
Node_205 node_205 = Node_205(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_206;
Node_206 node_206 = Node_206();

typedef xod_dev__ws2812__ws2812_device::Node<node_113_output_VAL> Node_207;
Node_207 node_207 = Node_207({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_208;
Node_208 node_208 = Node_208(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_209;
Node_209 node_209 = Node_209();

typedef xod_dev__ws2812__ws2812_device::Node<node_131_output_VAL> Node_210;
Node_210 node_210 = Node_210({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_211;
Node_211 node_211 = Node_211(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_212;
Node_212 node_212 = Node_212();

typedef xod_dev__ws2812__ws2812_device::Node<node_149_output_VAL> Node_213;
Node_213 node_213 = Node_213({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_214;
Node_214 node_214 = Node_214(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_215;
Node_215 node_215 = Node_215();

typedef xod_dev__ws2812__ws2812_device::Node<node_167_output_VAL> Node_216;
Node_216 node_216 = Node_216({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_217;
Node_217 node_217 = Node_217(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_218;
Node_218 node_218 = Node_218();

typedef xod_dev__ws2812__ws2812_device::Node<node_185_output_VAL> Node_219;
Node_219 node_219 = Node_219({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__core__throttle__color::Node Node_220;
Node_220 node_220 = Node_220(/* RGB */ { 0x00, 0x00, 0x00 });

typedef xod__core__pulse_on_change__number::Node Node_221;
Node_221 node_221 = Node_221();

typedef xod__gpio__digital_read_pullup::Node<node_11_output_VAL> Node_222;
Node_222 node_222 = Node_222(false);

typedef xod__gpio__digital_read_pullup::Node<node_13_output_VAL> Node_223;
Node_223 node_223 = Node_223(false);

typedef xod__gpio__digital_read_pullup::Node<node_15_output_VAL> Node_224;
Node_224 node_224 = Node_224(false);

typedef xod__gpio__digital_read_pullup::Node<node_17_output_VAL> Node_225;
Node_225 node_225 = Node_225(false);

typedef xod__gpio__digital_read_pullup::Node<node_19_output_VAL> Node_226;
Node_226 node_226 = Node_226(false);

typedef xod__gpio__digital_read_pullup::Node<node_21_output_VAL> Node_227;
Node_227 node_227 = Node_227(false);

typedef xod__gpio__digital_read_pullup::Node<node_119_output_VAL> Node_228;
Node_228 node_228 = Node_228(false);

typedef xod__gpio__digital_read_pullup::Node<node_137_output_VAL> Node_229;
Node_229 node_229 = Node_229(false);

typedef xod__gpio__digital_read_pullup::Node<node_155_output_VAL> Node_230;
Node_230 node_230 = Node_230(false);

typedef xod__gpio__digital_read_pullup::Node<node_173_output_VAL> Node_231;
Node_231 node_231 = Node_231(false);

typedef xod__color__pulse_on_change__color::Node Node_232;
Node_232 node_232 = Node_232();

typedef xod__core__any::Node Node_233;
Node_233 node_233 = Node_233();

typedef xod__color__pulse_on_change__color::Node Node_234;
Node_234 node_234 = Node_234();

typedef xod__core__any::Node Node_235;
Node_235 node_235 = Node_235();

typedef xod__color__pulse_on_change__color::Node Node_236;
Node_236 node_236 = Node_236();

typedef xod__core__any::Node Node_237;
Node_237 node_237 = Node_237();

typedef xod__color__pulse_on_change__color::Node Node_238;
Node_238 node_238 = Node_238();

typedef xod__core__any::Node Node_239;
Node_239 node_239 = Node_239();

typedef xod__color__pulse_on_change__color::Node Node_240;
Node_240 node_240 = Node_240();

typedef xod__core__any::Node Node_241;
Node_241 node_241 = Node_241();

typedef xod__color__pulse_on_change__color::Node Node_242;
Node_242 node_242 = Node_242();

typedef xod__core__any::Node Node_243;
Node_243 node_243 = Node_243();

typedef xod__color__pulse_on_change__color::Node Node_244;
Node_244 node_244 = Node_244();

typedef xod__core__any::Node Node_245;
Node_245 node_245 = Node_245();

typedef xod__color__pulse_on_change__color::Node Node_246;
Node_246 node_246 = Node_246();

typedef xod__core__any::Node Node_247;
Node_247 node_247 = Node_247();

typedef xod__color__pulse_on_change__color::Node Node_248;
Node_248 node_248 = Node_248();

typedef xod__core__any::Node Node_249;
Node_249 node_249 = Node_249();

typedef xod__color__pulse_on_change__color::Node Node_250;
Node_250 node_250 = Node_250();

typedef xod__core__any::Node Node_251;
Node_251 node_251 = Node_251();

typedef xod__core__not::Node Node_252;
Node_252 node_252 = Node_252(false);

typedef xod__core__not::Node Node_253;
Node_253 node_253 = Node_253(false);

typedef xod__core__not::Node Node_254;
Node_254 node_254 = Node_254(false);

typedef xod__core__not::Node Node_255;
Node_255 node_255 = Node_255(false);

typedef xod__core__not::Node Node_256;
Node_256 node_256 = Node_256(false);

typedef xod__core__not::Node Node_257;
Node_257 node_257 = Node_257(false);

typedef xod__core__not::Node Node_258;
Node_258 node_258 = Node_258(false);

typedef xod__core__not::Node Node_259;
Node_259 node_259 = Node_259(false);

typedef xod__core__not::Node Node_260;
Node_260 node_260 = Node_260(false);

typedef xod__core__not::Node Node_261;
Node_261 node_261 = Node_261(false);

typedef xod__core__any::Node Node_262;
Node_262 node_262 = Node_262();

typedef xod__core__any::Node Node_263;
Node_263 node_263 = Node_263();

typedef xod__core__any::Node Node_264;
Node_264 node_264 = Node_264();

typedef xod__core__any::Node Node_265;
Node_265 node_265 = Node_265();

typedef xod__core__any::Node Node_266;
Node_266 node_266 = Node_266();

typedef xod__core__any::Node Node_267;
Node_267 node_267 = Node_267();

typedef xod__core__any::Node Node_268;
Node_268 node_268 = Node_268();

typedef xod__core__any::Node Node_269;
Node_269 node_269 = Node_269();

typedef xod__core__any::Node Node_270;
Node_270 node_270 = Node_270();

typedef xod__core__any::Node Node_271;
Node_271 node_271 = Node_271();

typedef xod__core__debounce__boolean::Node Node_272;
Node_272 node_272 = Node_272(false);

typedef xod__core__debounce__boolean::Node Node_273;
Node_273 node_273 = Node_273(false);

typedef xod__core__debounce__boolean::Node Node_274;
Node_274 node_274 = Node_274(false);

typedef xod__core__debounce__boolean::Node Node_275;
Node_275 node_275 = Node_275(false);

typedef xod__core__debounce__boolean::Node Node_276;
Node_276 node_276 = Node_276(false);

typedef xod__core__debounce__boolean::Node Node_277;
Node_277 node_277 = Node_277(false);

typedef xod__core__debounce__boolean::Node Node_278;
Node_278 node_278 = Node_278(false);

typedef xod__core__debounce__boolean::Node Node_279;
Node_279 node_279 = Node_279(false);

typedef xod__core__debounce__boolean::Node Node_280;
Node_280 node_280 = Node_280(false);

typedef xod__core__debounce__boolean::Node Node_281;
Node_281 node_281 = Node_281(false);

typedef xod__core__not::Node Node_282;
Node_282 node_282 = Node_282(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_283;
Node_283 node_283 = Node_283();

typedef xod__core__cast_to_pulse__boolean::Node Node_284;
Node_284 node_284 = Node_284();

typedef xod__core__not::Node Node_285;
Node_285 node_285 = Node_285(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_286;
Node_286 node_286 = Node_286();

typedef xod__core__cast_to_pulse__boolean::Node Node_287;
Node_287 node_287 = Node_287();

typedef xod__core__not::Node Node_288;
Node_288 node_288 = Node_288(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_289;
Node_289 node_289 = Node_289();

typedef xod__core__cast_to_pulse__boolean::Node Node_290;
Node_290 node_290 = Node_290();

typedef xod__core__not::Node Node_291;
Node_291 node_291 = Node_291(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_292;
Node_292 node_292 = Node_292();

typedef xod__core__cast_to_pulse__boolean::Node Node_293;
Node_293 node_293 = Node_293();

typedef xod__core__not::Node Node_294;
Node_294 node_294 = Node_294(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_295;
Node_295 node_295 = Node_295();

typedef xod__core__cast_to_pulse__boolean::Node Node_296;
Node_296 node_296 = Node_296();

typedef xod__core__not::Node Node_297;
Node_297 node_297 = Node_297(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_298;
Node_298 node_298 = Node_298();

typedef xod__core__cast_to_pulse__boolean::Node Node_299;
Node_299 node_299 = Node_299();

typedef xod__core__not::Node Node_300;
Node_300 node_300 = Node_300(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_301;
Node_301 node_301 = Node_301();

typedef xod__core__cast_to_pulse__boolean::Node Node_302;
Node_302 node_302 = Node_302();

typedef xod__core__not::Node Node_303;
Node_303 node_303 = Node_303(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_304;
Node_304 node_304 = Node_304();

typedef xod__core__cast_to_pulse__boolean::Node Node_305;
Node_305 node_305 = Node_305();

typedef xod__core__not::Node Node_306;
Node_306 node_306 = Node_306(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_307;
Node_307 node_307 = Node_307();

typedef xod__core__cast_to_pulse__boolean::Node Node_308;
Node_308 node_308 = Node_308();

typedef xod__core__not::Node Node_309;
Node_309 node_309 = Node_309(false);

typedef xod__core__cast_to_pulse__boolean::Node Node_310;
Node_310 node_310 = Node_310();

typedef xod__core__cast_to_pulse__boolean::Node Node_311;
Node_311 node_311 = Node_311();

typedef xod__core__pulse_on_true::Node Node_312;
Node_312 node_312 = Node_312();

typedef xod__core__any::Node Node_313;
Node_313 node_313 = Node_313();

typedef xod__core__any::Node Node_314;
Node_314 node_314 = Node_314();

typedef xod__core__pulse_on_true::Node Node_315;
Node_315 node_315 = Node_315();

typedef xod__core__any::Node Node_316;
Node_316 node_316 = Node_316();

typedef xod__core__any::Node Node_317;
Node_317 node_317 = Node_317();

typedef xod__core__pulse_on_true::Node Node_318;
Node_318 node_318 = Node_318();

typedef xod__core__any::Node Node_319;
Node_319 node_319 = Node_319();

typedef xod__core__any::Node Node_320;
Node_320 node_320 = Node_320();

typedef xod__core__pulse_on_true::Node Node_321;
Node_321 node_321 = Node_321();

typedef xod__core__any::Node Node_322;
Node_322 node_322 = Node_322();

typedef xod__core__any::Node Node_323;
Node_323 node_323 = Node_323();

typedef xod__core__pulse_on_true::Node Node_324;
Node_324 node_324 = Node_324();

typedef xod__core__any::Node Node_325;
Node_325 node_325 = Node_325();

typedef xod__core__any::Node Node_326;
Node_326 node_326 = Node_326();

typedef xod__core__pulse_on_true::Node Node_327;
Node_327 node_327 = Node_327();

typedef xod__core__any::Node Node_328;
Node_328 node_328 = Node_328();

typedef xod__core__any::Node Node_329;
Node_329 node_329 = Node_329();

typedef xod__core__pulse_on_true::Node Node_330;
Node_330 node_330 = Node_330();

typedef xod__core__any::Node Node_331;
Node_331 node_331 = Node_331();

typedef xod__core__any::Node Node_332;
Node_332 node_332 = Node_332();

typedef xod__core__pulse_on_true::Node Node_333;
Node_333 node_333 = Node_333();

typedef xod__core__any::Node Node_334;
Node_334 node_334 = Node_334();

typedef xod__core__any::Node Node_335;
Node_335 node_335 = Node_335();

typedef xod__core__pulse_on_true::Node Node_336;
Node_336 node_336 = Node_336();

typedef xod__core__any::Node Node_337;
Node_337 node_337 = Node_337();

typedef xod__core__any::Node Node_338;
Node_338 node_338 = Node_338();

typedef xod__core__pulse_on_true::Node Node_339;
Node_339 node_339 = Node_339();

typedef xod__core__any::Node Node_340;
Node_340 node_340 = Node_340();

typedef xod__core__any::Node Node_341;
Node_341 node_341 = Node_341();

typedef xod__uart__soft_uart::Node<node_23_output_VAL, node_24_output_VAL> Node_342;
Node_342 node_342 = Node_342({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_343;
Node_343 node_343 = Node_343();

typedef xod__core__gate__pulse::Node Node_344;
Node_344 node_344 = Node_344();

typedef xod__uart__soft_uart::Node<node_34_output_VAL, node_35_output_VAL> Node_345;
Node_345 node_345 = Node_345({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_346;
Node_346 node_346 = Node_346();

typedef xod__core__gate__pulse::Node Node_347;
Node_347 node_347 = Node_347();

typedef xod__uart__soft_uart::Node<node_45_output_VAL, node_46_output_VAL> Node_348;
Node_348 node_348 = Node_348({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_349;
Node_349 node_349 = Node_349();

typedef xod__core__gate__pulse::Node Node_350;
Node_350 node_350 = Node_350();

typedef xod__uart__soft_uart::Node<node_56_output_VAL, node_57_output_VAL> Node_351;
Node_351 node_351 = Node_351({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_352;
Node_352 node_352 = Node_352();

typedef xod__core__gate__pulse::Node Node_353;
Node_353 node_353 = Node_353();

typedef xod__uart__soft_uart::Node<node_67_output_VAL, node_68_output_VAL> Node_354;
Node_354 node_354 = Node_354({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_355;
Node_355 node_355 = Node_355();

typedef xod__core__gate__pulse::Node Node_356;
Node_356 node_356 = Node_356();

typedef xod__uart__soft_uart::Node<node_78_output_VAL, node_79_output_VAL> Node_357;
Node_357 node_357 = Node_357({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_358;
Node_358 node_358 = Node_358();

typedef xod__core__gate__pulse::Node Node_359;
Node_359 node_359 = Node_359();

typedef xod__uart__soft_uart::Node<node_121_output_VAL, node_122_output_VAL> Node_360;
Node_360 node_360 = Node_360({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_361;
Node_361 node_361 = Node_361();

typedef xod__core__gate__pulse::Node Node_362;
Node_362 node_362 = Node_362();

typedef xod__uart__soft_uart::Node<node_139_output_VAL, node_140_output_VAL> Node_363;
Node_363 node_363 = Node_363({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_364;
Node_364 node_364 = Node_364();

typedef xod__core__gate__pulse::Node Node_365;
Node_365 node_365 = Node_365();

typedef xod__uart__soft_uart::Node<node_157_output_VAL, node_158_output_VAL> Node_366;
Node_366 node_366 = Node_366({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_367;
Node_367 node_367 = Node_367();

typedef xod__core__gate__pulse::Node Node_368;
Node_368 node_368 = Node_368();

typedef xod__uart__soft_uart::Node<node_175_output_VAL, node_176_output_VAL> Node_369;
Node_369 node_369 = Node_369({ /* xod/uart/uart */ });

typedef xod__core__gate__pulse::Node Node_370;
Node_370 node_370 = Node_370();

typedef xod__core__gate__pulse::Node Node_371;
Node_371 node_371 = Node_371();

typedef xod__uart__write_byte::Node Node_372;
Node_372 node_372 = Node_372();

typedef xod__core__any::Node Node_373;
Node_373 node_373 = Node_373();

typedef xod__uart__write_byte::Node Node_374;
Node_374 node_374 = Node_374();

typedef xod__core__any::Node Node_375;
Node_375 node_375 = Node_375();

typedef xod__uart__write_byte::Node Node_376;
Node_376 node_376 = Node_376();

typedef xod__core__any::Node Node_377;
Node_377 node_377 = Node_377();

typedef xod__uart__write_byte::Node Node_378;
Node_378 node_378 = Node_378();

typedef xod__core__any::Node Node_379;
Node_379 node_379 = Node_379();

typedef xod__uart__write_byte::Node Node_380;
Node_380 node_380 = Node_380();

typedef xod__core__any::Node Node_381;
Node_381 node_381 = Node_381();

typedef xod__uart__write_byte::Node Node_382;
Node_382 node_382 = Node_382();

typedef xod__core__any::Node Node_383;
Node_383 node_383 = Node_383();

typedef xod__uart__write_byte::Node Node_384;
Node_384 node_384 = Node_384();

typedef xod__core__any::Node Node_385;
Node_385 node_385 = Node_385();

typedef xod__uart__write_byte::Node Node_386;
Node_386 node_386 = Node_386();

typedef xod__core__any::Node Node_387;
Node_387 node_387 = Node_387();

typedef xod__uart__write_byte::Node Node_388;
Node_388 node_388 = Node_388();

typedef xod__core__any::Node Node_389;
Node_389 node_389 = Node_389();

typedef xod__uart__write_byte::Node Node_390;
Node_390 node_390 = Node_390();

typedef xod__core__any::Node Node_391;
Node_391 node_391 = Node_391();

typedef xod__uart__write_byte::Node Node_392;
Node_392 node_392 = Node_392();

typedef xod_dev__ws2812__fill_solid::Node<Node_192::typeof_DEV> Node_393;
Node_393 node_393 = Node_393({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_394;
Node_394 node_394 = Node_394();

typedef xod_dev__ws2812__fill_solid::Node<Node_195::typeof_DEV> Node_395;
Node_395 node_395 = Node_395({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_396;
Node_396 node_396 = Node_396();

typedef xod_dev__ws2812__fill_solid::Node<Node_207::typeof_DEV> Node_397;
Node_397 node_397 = Node_397({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_398;
Node_398 node_398 = Node_398();

typedef xod_dev__ws2812__fill_solid::Node<Node_198::typeof_DEV> Node_399;
Node_399 node_399 = Node_399({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_400;
Node_400 node_400 = Node_400();

typedef xod_dev__ws2812__fill_solid::Node<Node_201::typeof_DEV> Node_401;
Node_401 node_401 = Node_401({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_402;
Node_402 node_402 = Node_402();

typedef xod_dev__ws2812__fill_solid::Node<Node_204::typeof_DEV> Node_403;
Node_403 node_403 = Node_403({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_404;
Node_404 node_404 = Node_404();

typedef xod_dev__ws2812__fill_solid::Node<Node_210::typeof_DEV> Node_405;
Node_405 node_405 = Node_405({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_406;
Node_406 node_406 = Node_406();

typedef xod_dev__ws2812__fill_solid::Node<Node_213::typeof_DEV> Node_407;
Node_407 node_407 = Node_407({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_408;
Node_408 node_408 = Node_408();

typedef xod_dev__ws2812__fill_solid::Node<Node_216::typeof_DEV> Node_409;
Node_409 node_409 = Node_409({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_410;
Node_410 node_410 = Node_410();

typedef xod_dev__ws2812__fill_solid::Node<Node_219::typeof_DEV> Node_411;
Node_411 node_411 = Node_411({ /* xod-dev/ws2812/ws2812-device */ });

typedef xod__uart__write_byte::Node Node_412;
Node_412 node_412 = Node_412();

typedef xod__uart__write_byte::Node Node_413;
Node_413 node_413 = Node_413();

typedef xod__uart__write_byte::Node Node_414;
Node_414 node_414 = Node_414();

typedef xod__uart__write_byte::Node Node_415;
Node_415 node_415 = Node_415();

typedef xod__uart__write_byte::Node Node_416;
Node_416 node_416 = Node_416();

typedef xod__uart__write_byte::Node Node_417;
Node_417 node_417 = Node_417();

typedef xod__uart__write_byte::Node Node_418;
Node_418 node_418 = Node_418();

typedef xod__uart__write_byte::Node Node_419;
Node_419 node_419 = Node_419();

typedef xod__uart__write_byte::Node Node_420;
Node_420 node_420 = Node_420();

typedef xod__uart__write_byte::Node Node_421;
Node_421 node_421 = Node_421();

typedef xod__uart__write_byte::Node Node_422;
Node_422 node_422 = Node_422();

typedef xod__uart__write_byte::Node Node_423;
Node_423 node_423 = Node_423();

typedef xod__uart__write_byte::Node Node_424;
Node_424 node_424 = Node_424();

typedef xod__uart__write_byte::Node Node_425;
Node_425 node_425 = Node_425();

typedef xod__uart__write_byte::Node Node_426;
Node_426 node_426 = Node_426();

typedef xod__uart__write_byte::Node Node_427;
Node_427 node_427 = Node_427();

typedef xod__uart__write_byte::Node Node_428;
Node_428 node_428 = Node_428();

typedef xod__uart__write_byte::Node Node_429;
Node_429 node_429 = Node_429();

typedef xod__uart__write_byte::Node Node_430;
Node_430 node_430 = Node_430();

typedef xod__uart__write_byte::Node Node_431;
Node_431 node_431 = Node_431();

typedef xod__uart__write_byte::Node Node_432;
Node_432 node_432 = Node_432();

typedef xod__uart__write_byte::Node Node_433;
Node_433 node_433 = Node_433();

typedef xod__uart__write_byte::Node Node_434;
Node_434 node_434 = Node_434();

typedef xod__uart__write_byte::Node Node_435;
Node_435 node_435 = Node_435();

typedef xod__uart__write_byte::Node Node_436;
Node_436 node_436 = Node_436();

typedef xod__uart__write_byte::Node Node_437;
Node_437 node_437 = Node_437();

typedef xod__uart__write_byte::Node Node_438;
Node_438 node_438 = Node_438();

typedef xod__uart__write_byte::Node Node_439;
Node_439 node_439 = Node_439();

typedef xod__uart__write_byte::Node Node_440;
Node_440 node_440 = Node_440();

typedef xod__uart__write_byte::Node Node_441;
Node_441 node_441 = Node_441();

typedef xod__uart__write_byte::Node Node_442;
Node_442 node_442 = Node_442();

typedef xod__uart__write_byte::Node Node_443;
Node_443 node_443 = Node_443();

typedef xod__uart__write_byte::Node Node_444;
Node_444 node_444 = Node_444();

typedef xod__uart__write_byte::Node Node_445;
Node_445 node_445 = Node_445();

typedef xod__uart__write_byte::Node Node_446;
Node_446 node_446 = Node_446();

typedef xod__uart__write_byte::Node Node_447;
Node_447 node_447 = Node_447();

typedef xod__uart__write_byte::Node Node_448;
Node_448 node_448 = Node_448();

typedef xod__uart__write_byte::Node Node_449;
Node_449 node_449 = Node_449();

typedef xod__uart__write_byte::Node Node_450;
Node_450 node_450 = Node_450();

typedef xod__uart__write_byte::Node Node_451;
Node_451 node_451 = Node_451();

typedef xod__uart__write_byte::Node Node_452;
Node_452 node_452 = Node_452();

typedef xod__uart__write_byte::Node Node_453;
Node_453 node_453 = Node_453();

typedef xod__uart__write_byte::Node Node_454;
Node_454 node_454 = Node_454();

typedef xod__uart__write_byte::Node Node_455;
Node_455 node_455 = Node_455();

typedef xod__uart__write_byte::Node Node_456;
Node_456 node_456 = Node_456();

typedef xod__uart__write_byte::Node Node_457;
Node_457 node_457 = Node_457();

typedef xod__uart__write_byte::Node Node_458;
Node_458 node_458 = Node_458();

typedef xod__uart__write_byte::Node Node_459;
Node_459 node_459 = Node_459();

typedef xod__uart__write_byte::Node Node_460;
Node_460 node_460 = Node_460();

typedef xod__uart__write_byte::Node Node_461;
Node_461 node_461 = Node_461();

typedef xod__uart__write_byte::Node Node_462;
Node_462 node_462 = Node_462();

typedef xod__uart__write_byte::Node Node_463;
Node_463 node_463 = Node_463();

typedef xod__uart__write_byte::Node Node_464;
Node_464 node_464 = Node_464();

typedef xod__uart__write_byte::Node Node_465;
Node_465 node_465 = Node_465();

typedef xod__uart__write_byte::Node Node_466;
Node_466 node_466 = Node_466();

typedef xod__uart__write_byte::Node Node_467;
Node_467 node_467 = Node_467();

typedef xod__uart__write_byte::Node Node_468;
Node_468 node_468 = Node_468();

typedef xod__uart__write_byte::Node Node_469;
Node_469 node_469 = Node_469();

typedef xod__uart__write_byte::Node Node_470;
Node_470 node_470 = Node_470();

typedef xod__uart__write_byte::Node Node_471;
Node_471 node_471 = Node_471();

typedef xod__uart__end::Node Node_472;
Node_472 node_472 = Node_472();

typedef xod__uart__end::Node Node_473;
Node_473 node_473 = Node_473();

typedef xod__uart__end::Node Node_474;
Node_474 node_474 = Node_474();

typedef xod__uart__end::Node Node_475;
Node_475 node_475 = Node_475();

typedef xod__uart__end::Node Node_476;
Node_476 node_476 = Node_476();

typedef xod__uart__end::Node Node_477;
Node_477 node_477 = Node_477();

typedef xod__uart__end::Node Node_478;
Node_478 node_478 = Node_478();

typedef xod__uart__end::Node Node_479;
Node_479 node_479 = Node_479();

typedef xod__uart__end::Node Node_480;
Node_480 node_480 = Node_480();

typedef xod__uart__end::Node Node_481;
Node_481 node_481 = Node_481();

typedef xod__core__defer__pulse::Node Node_482;
Node_482 node_482 = Node_482();

typedef xod__core__defer__pulse::Node Node_483;
Node_483 node_483 = Node_483();

typedef xod__core__defer__pulse::Node Node_484;
Node_484 node_484 = Node_484();

typedef xod__core__defer__pulse::Node Node_485;
Node_485 node_485 = Node_485();

typedef xod__core__defer__pulse::Node Node_486;
Node_486 node_486 = Node_486();

typedef xod__core__defer__pulse::Node Node_487;
Node_487 node_487 = Node_487();

typedef xod__core__defer__pulse::Node Node_488;
Node_488 node_488 = Node_488();

typedef xod__core__defer__pulse::Node Node_489;
Node_489 node_489 = Node_489();

typedef xod__core__defer__pulse::Node Node_490;
Node_490 node_490 = Node_490();

typedef xod__core__defer__pulse::Node Node_491;
Node_491 node_491 = Node_491();

#if defined(XOD_DEBUG) || defined(XOD_SIMULATION)
namespace detail {
void handleDebugProtocolMessages() {
    bool rewindToEol = true;

    if (
      XOD_DEBUG_SERIAL.available() > 0 &&
      XOD_DEBUG_SERIAL.find("+XOD:", 5)
    ) {
        int tweakedNodeId = XOD_DEBUG_SERIAL.parseInt();

        switch (tweakedNodeId) {
            case 0:
                {
                    node_0._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_0_isNodeDirty = true;
                    g_transaction.node_0_isOutputDirty_OUT = true;
                }
                break;
            case 1:
                {
                    node_1._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_1_isNodeDirty = true;
                    g_transaction.node_1_isOutputDirty_OUT = true;
                }
                break;
            case 2:
                {
                    node_2._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_2_isNodeDirty = true;
                    g_transaction.node_2_isOutputDirty_OUT = true;
                }
                break;
            case 3:
                {
                    node_3._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_3_isNodeDirty = true;
                    g_transaction.node_3_isOutputDirty_OUT = true;
                }
                break;
            case 4:
                {
                    node_4._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_4_isNodeDirty = true;
                    g_transaction.node_4_isOutputDirty_OUT = true;
                }
                break;
            case 5:
                {
                    node_5._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_5_isNodeDirty = true;
                    g_transaction.node_5_isOutputDirty_OUT = true;
                }
                break;
            case 6:
                {
                    node_6._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_6_isNodeDirty = true;
                    g_transaction.node_6_isOutputDirty_OUT = true;
                }
                break;
            case 7:
                {
                    node_7._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_7_isNodeDirty = true;
                    g_transaction.node_7_isOutputDirty_OUT = true;
                }
                break;
            case 8:
                {
                    node_8._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_8_isNodeDirty = true;
                    g_transaction.node_8_isOutputDirty_OUT = true;
                }
                break;
            case 9:
                {
                    node_9._output_OUT = {
                      /* RGB */
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt(),
                      (uint8_t)XOD_DEBUG_SERIAL.parseInt()
                    };
                    rewindToEol = true;
                    // to run evaluate and mark all downstream nodes as dirty
                    g_transaction.node_9_isNodeDirty = true;
                    g_transaction.node_9_isOutputDirty_OUT = true;
                }
                break;
        }

        if (rewindToEol)
            XOD_DEBUG_SERIAL.find('\n');
    }
}
} // namespace detail
#endif

// Copy values bound to `tweak-string`s outputs
// directly into buffers instead of wasting memory
// on XStringCString with initial values
void initializeTweakStrings() {
}

void handleDefers() {
    {
        if (g_transaction.node_482_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_462.errors.output_DONE;
            error_input_IN |= node_452.errors.output_DONE;
            error_input_IN |= node_442.errors.output_DONE;
            error_input_IN |= node_432.errors.output_DONE;
            error_input_IN |= node_422.errors.output_DONE;
            error_input_IN |= node_412.errors.output_DONE;
            error_input_IN |= node_392.errors.output_DONE;
            error_input_IN |= node_372.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(482);

            Node_482::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_482::NodeErrors previousErrors = node_482.errors;

            node_482.errors.output_OUT = false;

            node_482.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_482.errors.flags) {
                detail::printErrorToDebugSerial(482, node_482.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_482.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_482.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_482_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_482.errors.flags) {
            if (node_482.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_483_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_463.errors.output_DONE;
            error_input_IN |= node_453.errors.output_DONE;
            error_input_IN |= node_443.errors.output_DONE;
            error_input_IN |= node_433.errors.output_DONE;
            error_input_IN |= node_423.errors.output_DONE;
            error_input_IN |= node_413.errors.output_DONE;
            error_input_IN |= node_394.errors.output_DONE;
            error_input_IN |= node_374.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(483);

            Node_483::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_483::NodeErrors previousErrors = node_483.errors;

            node_483.errors.output_OUT = false;

            node_483.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_483.errors.flags) {
                detail::printErrorToDebugSerial(483, node_483.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_483.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_483.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_483_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_483.errors.flags) {
            if (node_483.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_484_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_464.errors.output_DONE;
            error_input_IN |= node_454.errors.output_DONE;
            error_input_IN |= node_444.errors.output_DONE;
            error_input_IN |= node_434.errors.output_DONE;
            error_input_IN |= node_424.errors.output_DONE;
            error_input_IN |= node_414.errors.output_DONE;
            error_input_IN |= node_396.errors.output_DONE;
            error_input_IN |= node_376.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(484);

            Node_484::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_484::NodeErrors previousErrors = node_484.errors;

            node_484.errors.output_OUT = false;

            node_484.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_484.errors.flags) {
                detail::printErrorToDebugSerial(484, node_484.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_484.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_484.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_484_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_484.errors.flags) {
            if (node_484.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_485_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_465.errors.output_DONE;
            error_input_IN |= node_455.errors.output_DONE;
            error_input_IN |= node_445.errors.output_DONE;
            error_input_IN |= node_435.errors.output_DONE;
            error_input_IN |= node_425.errors.output_DONE;
            error_input_IN |= node_415.errors.output_DONE;
            error_input_IN |= node_398.errors.output_DONE;
            error_input_IN |= node_378.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(485);

            Node_485::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_485::NodeErrors previousErrors = node_485.errors;

            node_485.errors.output_OUT = false;

            node_485.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_485.errors.flags) {
                detail::printErrorToDebugSerial(485, node_485.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_485.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_485.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_485_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_485.errors.flags) {
            if (node_485.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_486_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_466.errors.output_DONE;
            error_input_IN |= node_456.errors.output_DONE;
            error_input_IN |= node_446.errors.output_DONE;
            error_input_IN |= node_436.errors.output_DONE;
            error_input_IN |= node_426.errors.output_DONE;
            error_input_IN |= node_416.errors.output_DONE;
            error_input_IN |= node_400.errors.output_DONE;
            error_input_IN |= node_380.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(486);

            Node_486::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_486::NodeErrors previousErrors = node_486.errors;

            node_486.errors.output_OUT = false;

            node_486.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_486.errors.flags) {
                detail::printErrorToDebugSerial(486, node_486.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_486.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_486.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_486_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_486.errors.flags) {
            if (node_486.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_487_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_467.errors.output_DONE;
            error_input_IN |= node_457.errors.output_DONE;
            error_input_IN |= node_447.errors.output_DONE;
            error_input_IN |= node_437.errors.output_DONE;
            error_input_IN |= node_427.errors.output_DONE;
            error_input_IN |= node_417.errors.output_DONE;
            error_input_IN |= node_402.errors.output_DONE;
            error_input_IN |= node_382.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(487);

            Node_487::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_487::NodeErrors previousErrors = node_487.errors;

            node_487.errors.output_OUT = false;

            node_487.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_487.errors.flags) {
                detail::printErrorToDebugSerial(487, node_487.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_487.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_487.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_487_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_487.errors.flags) {
            if (node_487.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_488_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_468.errors.output_DONE;
            error_input_IN |= node_458.errors.output_DONE;
            error_input_IN |= node_448.errors.output_DONE;
            error_input_IN |= node_438.errors.output_DONE;
            error_input_IN |= node_428.errors.output_DONE;
            error_input_IN |= node_418.errors.output_DONE;
            error_input_IN |= node_404.errors.output_DONE;
            error_input_IN |= node_384.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(488);

            Node_488::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_488::NodeErrors previousErrors = node_488.errors;

            node_488.errors.output_OUT = false;

            node_488.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_488.errors.flags) {
                detail::printErrorToDebugSerial(488, node_488.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_488.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_488.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_488_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_488.errors.flags) {
            if (node_488.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_489_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_469.errors.output_DONE;
            error_input_IN |= node_459.errors.output_DONE;
            error_input_IN |= node_449.errors.output_DONE;
            error_input_IN |= node_439.errors.output_DONE;
            error_input_IN |= node_429.errors.output_DONE;
            error_input_IN |= node_419.errors.output_DONE;
            error_input_IN |= node_406.errors.output_DONE;
            error_input_IN |= node_386.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(489);

            Node_489::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_489::NodeErrors previousErrors = node_489.errors;

            node_489.errors.output_OUT = false;

            node_489.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_489.errors.flags) {
                detail::printErrorToDebugSerial(489, node_489.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_489.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_489.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_489_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_489.errors.flags) {
            if (node_489.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_490_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_470.errors.output_DONE;
            error_input_IN |= node_460.errors.output_DONE;
            error_input_IN |= node_450.errors.output_DONE;
            error_input_IN |= node_440.errors.output_DONE;
            error_input_IN |= node_430.errors.output_DONE;
            error_input_IN |= node_420.errors.output_DONE;
            error_input_IN |= node_408.errors.output_DONE;
            error_input_IN |= node_388.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(490);

            Node_490::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_490::NodeErrors previousErrors = node_490.errors;

            node_490.errors.output_OUT = false;

            node_490.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_490.errors.flags) {
                detail::printErrorToDebugSerial(490, node_490.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_490.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_490.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_490_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_490.errors.flags) {
            if (node_490.errors.output_OUT) {
            }
        }
    }
    {
        if (g_transaction.node_491_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_471.errors.output_DONE;
            error_input_IN |= node_461.errors.output_DONE;
            error_input_IN |= node_451.errors.output_DONE;
            error_input_IN |= node_441.errors.output_DONE;
            error_input_IN |= node_431.errors.output_DONE;
            error_input_IN |= node_421.errors.output_DONE;
            error_input_IN |= node_410.errors.output_DONE;
            error_input_IN |= node_390.errors.output_DONE;

            XOD_TRACE_F("Trigger defer node #");
            XOD_TRACE_LN(491);

            Node_491::ContextObject ctxObj;
            ctxObj._isInputDirty_IN = false;

            ctxObj._error_input_IN = error_input_IN;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_491::NodeErrors previousErrors = node_491.errors;

            node_491.errors.output_OUT = false;

            node_491.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_491.errors.flags) {
                detail::printErrorToDebugSerial(491, node_491.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_491.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_491.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty

            g_transaction.node_491_isNodeDirty = false;
        }

        // propagate the error hold by the defer node
        if (node_491.errors.flags) {
            if (node_491.errors.output_OUT) {
            }
        }
    }
}

void runTransaction() {
    g_transactionTime = millis();

    XOD_TRACE_F("Transaction started, t=");
    XOD_TRACE_LN(g_transactionTime);

#if defined(XOD_DEBUG) || defined(XOD_SIMULATION)
    detail::handleDebugProtocolMessages();
#endif

    // Check for timeouts
    g_transaction.node_193_isNodeDirty |= detail::isTimedOut(&node_193);
    g_transaction.node_196_isNodeDirty |= detail::isTimedOut(&node_196);
    g_transaction.node_199_isNodeDirty |= detail::isTimedOut(&node_199);
    g_transaction.node_202_isNodeDirty |= detail::isTimedOut(&node_202);
    g_transaction.node_205_isNodeDirty |= detail::isTimedOut(&node_205);
    g_transaction.node_208_isNodeDirty |= detail::isTimedOut(&node_208);
    g_transaction.node_211_isNodeDirty |= detail::isTimedOut(&node_211);
    g_transaction.node_214_isNodeDirty |= detail::isTimedOut(&node_214);
    g_transaction.node_217_isNodeDirty |= detail::isTimedOut(&node_217);
    g_transaction.node_220_isNodeDirty |= detail::isTimedOut(&node_220);
    g_transaction.node_272_isNodeDirty |= detail::isTimedOut(&node_272);
    g_transaction.node_273_isNodeDirty |= detail::isTimedOut(&node_273);
    g_transaction.node_274_isNodeDirty |= detail::isTimedOut(&node_274);
    g_transaction.node_275_isNodeDirty |= detail::isTimedOut(&node_275);
    g_transaction.node_276_isNodeDirty |= detail::isTimedOut(&node_276);
    g_transaction.node_277_isNodeDirty |= detail::isTimedOut(&node_277);
    g_transaction.node_278_isNodeDirty |= detail::isTimedOut(&node_278);
    g_transaction.node_279_isNodeDirty |= detail::isTimedOut(&node_279);
    g_transaction.node_280_isNodeDirty |= detail::isTimedOut(&node_280);
    g_transaction.node_281_isNodeDirty |= detail::isTimedOut(&node_281);
    if (node_190.isSetImmediate) {
      node_190.isSetImmediate = false;
      g_transaction.node_190_isNodeDirty = true;
    }
    if (node_482.isSetImmediate) {
      node_482.isSetImmediate = false;
      g_transaction.node_482_isNodeDirty = true;
    }
    if (node_483.isSetImmediate) {
      node_483.isSetImmediate = false;
      g_transaction.node_483_isNodeDirty = true;
    }
    if (node_484.isSetImmediate) {
      node_484.isSetImmediate = false;
      g_transaction.node_484_isNodeDirty = true;
    }
    if (node_485.isSetImmediate) {
      node_485.isSetImmediate = false;
      g_transaction.node_485_isNodeDirty = true;
    }
    if (node_486.isSetImmediate) {
      node_486.isSetImmediate = false;
      g_transaction.node_486_isNodeDirty = true;
    }
    if (node_487.isSetImmediate) {
      node_487.isSetImmediate = false;
      g_transaction.node_487_isNodeDirty = true;
    }
    if (node_488.isSetImmediate) {
      node_488.isSetImmediate = false;
      g_transaction.node_488_isNodeDirty = true;
    }
    if (node_489.isSetImmediate) {
      node_489.isSetImmediate = false;
      g_transaction.node_489_isNodeDirty = true;
    }
    if (node_490.isSetImmediate) {
      node_490.isSetImmediate = false;
      g_transaction.node_490_isNodeDirty = true;
    }
    if (node_491.isSetImmediate) {
      node_491.isSetImmediate = false;
      g_transaction.node_491_isNodeDirty = true;
    }

    if (isSettingUp()) {
        initializeTweakStrings();
    } else {
        // defer-* nodes are always at the very bottom of the graph, so no one will
        // recieve values emitted by them. We must evaluate them before everybody
        // else to give them a chance to emit values.
        //
        // If trigerred, keep only output dirty, not the node itself, so it will
        // evaluate on the regular pass only if it receives a new value again.
        g_isEarlyDeferPass = true;
        handleDefers();
        g_isEarlyDeferPass = false;
    }

    // Evaluate all dirty nodes
    { // xod__debug__tweak_color #0
        if (g_transaction.node_0_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(0);

            Node_0::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_0_isOutputDirty_OUT;

            node_0.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_0_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_193_isNodeDirty |= g_transaction.node_0_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #1
        if (g_transaction.node_1_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(1);

            Node_1::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_1_isOutputDirty_OUT;

            node_1.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_1_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_196_isNodeDirty |= g_transaction.node_1_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #2
        if (g_transaction.node_2_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(2);

            Node_2::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_2_isOutputDirty_OUT;

            node_2.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_2_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_199_isNodeDirty |= g_transaction.node_2_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #3
        if (g_transaction.node_3_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(3);

            Node_3::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_3_isOutputDirty_OUT;

            node_3.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_3_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_202_isNodeDirty |= g_transaction.node_3_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #4
        if (g_transaction.node_4_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(4);

            Node_4::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_4_isOutputDirty_OUT;

            node_4.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_4_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_205_isNodeDirty |= g_transaction.node_4_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #5
        if (g_transaction.node_5_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(5);

            Node_5::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_5_isOutputDirty_OUT;

            node_5.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_5_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_208_isNodeDirty |= g_transaction.node_5_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #6
        if (g_transaction.node_6_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(6);

            Node_6::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_6_isOutputDirty_OUT;

            node_6.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_6_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_211_isNodeDirty |= g_transaction.node_6_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #7
        if (g_transaction.node_7_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(7);

            Node_7::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_7_isOutputDirty_OUT;

            node_7.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_7_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_214_isNodeDirty |= g_transaction.node_7_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #8
        if (g_transaction.node_8_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(8);

            Node_8::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_8_isOutputDirty_OUT;

            node_8.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_8_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_217_isNodeDirty |= g_transaction.node_8_isOutputDirty_OUT;
        }

    }
    { // xod__debug__tweak_color #9
        if (g_transaction.node_9_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(9);

            Node_9::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = g_transaction.node_9_isOutputDirty_OUT;

            node_9.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_9_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_220_isNodeDirty |= g_transaction.node_9_isOutputDirty_OUT;
        }

    }
    { // xod__core__continuously #190
        if (g_transaction.node_190_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(190);

            Node_190::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_TICK = false;

            node_190.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_190_isOutputDirty_TICK = ctxObj._isOutputDirty_TICK;

            // mark downstream nodes dirty
            g_transaction.node_222_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_223_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_224_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_225_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_226_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_227_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_228_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_229_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_230_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
            g_transaction.node_231_isNodeDirty |= g_transaction.node_190_isOutputDirty_TICK;
        }

    }
    { // xod__core__boot #191
        if (g_transaction.node_191_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(191);

            Node_191::ContextObject ctxObj;

            // copy data from upstream nodes into context

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_BOOT = false;

            node_191.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_191_isOutputDirty_BOOT = ctxObj._isOutputDirty_BOOT;

            // mark downstream nodes dirty
            g_transaction.node_233_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_235_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_237_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_239_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_241_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_243_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_245_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_247_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_249_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_251_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_262_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_263_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_264_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_265_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_266_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_267_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_268_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_269_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_270_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
            g_transaction.node_271_isNodeDirty |= g_transaction.node_191_isOutputDirty_BOOT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #192
        if (g_transaction.node_192_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(192);

            Node_192::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_89_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_192.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_192_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_393_isNodeDirty |= g_transaction.node_192_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #193
        if (g_transaction.node_193_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(193);

            Node_193::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_0._output_OUT;
            ctxObj._input_T = node_90_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_0_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_193.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_193_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_232_isNodeDirty |= g_transaction.node_193_isOutputDirty_OUT;
            g_transaction.node_393_isNodeDirty |= g_transaction.node_193_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #194
        if (g_transaction.node_194_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(194);

            Node_194::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_92_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_194.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_194_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_233_isNodeDirty |= g_transaction.node_194_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #195
        if (g_transaction.node_195_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(195);

            Node_195::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_94_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_195.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_195_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_395_isNodeDirty |= g_transaction.node_195_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #196
        if (g_transaction.node_196_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(196);

            Node_196::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_1._output_OUT;
            ctxObj._input_T = node_95_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_1_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_196.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_196_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_234_isNodeDirty |= g_transaction.node_196_isOutputDirty_OUT;
            g_transaction.node_395_isNodeDirty |= g_transaction.node_196_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #197
        if (g_transaction.node_197_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(197);

            Node_197::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_97_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_197.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_197_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_235_isNodeDirty |= g_transaction.node_197_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #198
        if (g_transaction.node_198_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(198);

            Node_198::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_99_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_198.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_198_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_399_isNodeDirty |= g_transaction.node_198_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #199
        if (g_transaction.node_199_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(199);

            Node_199::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_2._output_OUT;
            ctxObj._input_T = node_100_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_2_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_199.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_199_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_236_isNodeDirty |= g_transaction.node_199_isOutputDirty_OUT;
            g_transaction.node_399_isNodeDirty |= g_transaction.node_199_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #200
        if (g_transaction.node_200_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(200);

            Node_200::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_102_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_200.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_200_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_237_isNodeDirty |= g_transaction.node_200_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #201
        if (g_transaction.node_201_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(201);

            Node_201::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_104_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_201.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_201_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_401_isNodeDirty |= g_transaction.node_201_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #202
        if (g_transaction.node_202_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(202);

            Node_202::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_3._output_OUT;
            ctxObj._input_T = node_105_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_3_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_202.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_202_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_238_isNodeDirty |= g_transaction.node_202_isOutputDirty_OUT;
            g_transaction.node_401_isNodeDirty |= g_transaction.node_202_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #203
        if (g_transaction.node_203_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(203);

            Node_203::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_107_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_203.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_203_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_239_isNodeDirty |= g_transaction.node_203_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #204
        if (g_transaction.node_204_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(204);

            Node_204::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_109_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_204.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_204_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_403_isNodeDirty |= g_transaction.node_204_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #205
        if (g_transaction.node_205_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(205);

            Node_205::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_4._output_OUT;
            ctxObj._input_T = node_110_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_4_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_205.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_205_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_240_isNodeDirty |= g_transaction.node_205_isOutputDirty_OUT;
            g_transaction.node_403_isNodeDirty |= g_transaction.node_205_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #206
        if (g_transaction.node_206_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(206);

            Node_206::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_112_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_206.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_206_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_241_isNodeDirty |= g_transaction.node_206_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #207
        if (g_transaction.node_207_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(207);

            Node_207::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_114_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_207.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_207_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_397_isNodeDirty |= g_transaction.node_207_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #208
        if (g_transaction.node_208_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(208);

            Node_208::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_5._output_OUT;
            ctxObj._input_T = node_115_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_5_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_208.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_208_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_242_isNodeDirty |= g_transaction.node_208_isOutputDirty_OUT;
            g_transaction.node_397_isNodeDirty |= g_transaction.node_208_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #209
        if (g_transaction.node_209_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(209);

            Node_209::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_117_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_209.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_209_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_243_isNodeDirty |= g_transaction.node_209_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #210
        if (g_transaction.node_210_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(210);

            Node_210::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_132_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_210.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_210_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_405_isNodeDirty |= g_transaction.node_210_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #211
        if (g_transaction.node_211_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(211);

            Node_211::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_6._output_OUT;
            ctxObj._input_T = node_133_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_6_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_211.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_211_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_244_isNodeDirty |= g_transaction.node_211_isOutputDirty_OUT;
            g_transaction.node_405_isNodeDirty |= g_transaction.node_211_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #212
        if (g_transaction.node_212_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(212);

            Node_212::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_135_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_212.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_212_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_245_isNodeDirty |= g_transaction.node_212_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #213
        if (g_transaction.node_213_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(213);

            Node_213::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_150_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_213.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_213_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_407_isNodeDirty |= g_transaction.node_213_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #214
        if (g_transaction.node_214_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(214);

            Node_214::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_7._output_OUT;
            ctxObj._input_T = node_151_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_7_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_214.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_214_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_246_isNodeDirty |= g_transaction.node_214_isOutputDirty_OUT;
            g_transaction.node_407_isNodeDirty |= g_transaction.node_214_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #215
        if (g_transaction.node_215_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(215);

            Node_215::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_153_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_215.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_215_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_247_isNodeDirty |= g_transaction.node_215_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #216
        if (g_transaction.node_216_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(216);

            Node_216::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_168_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_216.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_216_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_409_isNodeDirty |= g_transaction.node_216_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #217
        if (g_transaction.node_217_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(217);

            Node_217::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_8._output_OUT;
            ctxObj._input_T = node_169_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_8_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_217.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_217_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_248_isNodeDirty |= g_transaction.node_217_isOutputDirty_OUT;
            g_transaction.node_409_isNodeDirty |= g_transaction.node_217_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #218
        if (g_transaction.node_218_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(218);

            Node_218::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_171_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_218.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_218_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_249_isNodeDirty |= g_transaction.node_218_isOutputDirty_OUT;
        }

    }
    { // xod_dev__ws2812__ws2812_device #219
        if (g_transaction.node_219_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(219);

            Node_219::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_W = node_186_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEV = false;

            node_219.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_219_isOutputDirty_DEV = ctxObj._isOutputDirty_DEV;

            // mark downstream nodes dirty
            g_transaction.node_411_isNodeDirty |= g_transaction.node_219_isOutputDirty_DEV;
        }

    }
    { // xod__core__throttle__color #220
        if (g_transaction.node_220_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(220);

            Node_220::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_9._output_OUT;
            ctxObj._input_T = node_187_output_VAL;

            ctxObj._isInputDirty_IN = g_transaction.node_9_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_220.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_220_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_250_isNodeDirty |= g_transaction.node_220_isOutputDirty_OUT;
            g_transaction.node_411_isNodeDirty |= g_transaction.node_220_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_change__number #221
        if (g_transaction.node_221_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(221);

            Node_221::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_189_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_221.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_221_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_251_isNodeDirty |= g_transaction.node_221_isOutputDirty_OUT;
        }

    }
    { // xod__gpio__digital_read_pullup #222
        if (g_transaction.node_222_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(222);

            Node_222::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_222.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_222_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_252_isNodeDirty |= g_transaction.node_222_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #223
        if (g_transaction.node_223_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(223);

            Node_223::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_223.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_223_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_253_isNodeDirty |= g_transaction.node_223_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #224
        if (g_transaction.node_224_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(224);

            Node_224::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_224.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_224_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_254_isNodeDirty |= g_transaction.node_224_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #225
        if (g_transaction.node_225_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(225);

            Node_225::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_225.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_225_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_255_isNodeDirty |= g_transaction.node_225_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #226
        if (g_transaction.node_226_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(226);

            Node_226::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_226.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_226_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_256_isNodeDirty |= g_transaction.node_226_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #227
        if (g_transaction.node_227_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(227);

            Node_227::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_227.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_227_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_257_isNodeDirty |= g_transaction.node_227_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #228
        if (g_transaction.node_228_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(228);

            Node_228::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_228.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_228_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_258_isNodeDirty |= g_transaction.node_228_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #229
        if (g_transaction.node_229_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(229);

            Node_229::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_229.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_229_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_259_isNodeDirty |= g_transaction.node_229_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #230
        if (g_transaction.node_230_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(230);

            Node_230::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_230.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_230_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_260_isNodeDirty |= g_transaction.node_230_isOutputDirty_SIG;
        }

    }
    { // xod__gpio__digital_read_pullup #231
        if (g_transaction.node_231_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(231);

            Node_231::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_UPD = g_transaction.node_190_isOutputDirty_TICK;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_SIG = false;
            ctxObj._isOutputDirty_DONE = false;

            node_231.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_231_isOutputDirty_SIG = ctxObj._isOutputDirty_SIG;

            // mark downstream nodes dirty
            g_transaction.node_261_isNodeDirty |= g_transaction.node_231_isOutputDirty_SIG;
        }

    }
    { // xod__color__pulse_on_change__color #232
        if (g_transaction.node_232_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(232);

            Node_232::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_193._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_232.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_232_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_262_isNodeDirty |= g_transaction.node_232_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #233
        if (g_transaction.node_233_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(233);

            Node_233::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_194_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_233.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_233_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_314_isNodeDirty |= g_transaction.node_233_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #234
        if (g_transaction.node_234_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(234);

            Node_234::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_196._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_234.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_234_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_263_isNodeDirty |= g_transaction.node_234_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #235
        if (g_transaction.node_235_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(235);

            Node_235::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_197_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_235.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_235_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_317_isNodeDirty |= g_transaction.node_235_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #236
        if (g_transaction.node_236_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(236);

            Node_236::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_199._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_236.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_236_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_264_isNodeDirty |= g_transaction.node_236_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #237
        if (g_transaction.node_237_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(237);

            Node_237::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_200_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_237.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_237_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_323_isNodeDirty |= g_transaction.node_237_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #238
        if (g_transaction.node_238_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(238);

            Node_238::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_202._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_238.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_238_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_265_isNodeDirty |= g_transaction.node_238_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #239
        if (g_transaction.node_239_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(239);

            Node_239::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_203_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_239.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_239_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_326_isNodeDirty |= g_transaction.node_239_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #240
        if (g_transaction.node_240_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(240);

            Node_240::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_205._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_240.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_240_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_266_isNodeDirty |= g_transaction.node_240_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #241
        if (g_transaction.node_241_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(241);

            Node_241::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_206_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_241.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_241_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_329_isNodeDirty |= g_transaction.node_241_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #242
        if (g_transaction.node_242_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(242);

            Node_242::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_208._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_242.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_242_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_267_isNodeDirty |= g_transaction.node_242_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #243
        if (g_transaction.node_243_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(243);

            Node_243::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_209_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_243.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_243_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_320_isNodeDirty |= g_transaction.node_243_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #244
        if (g_transaction.node_244_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(244);

            Node_244::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_211._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_244.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_244_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_268_isNodeDirty |= g_transaction.node_244_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #245
        if (g_transaction.node_245_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(245);

            Node_245::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_212_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_245.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_245_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_332_isNodeDirty |= g_transaction.node_245_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #246
        if (g_transaction.node_246_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(246);

            Node_246::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_214._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_246.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_246_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_269_isNodeDirty |= g_transaction.node_246_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #247
        if (g_transaction.node_247_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(247);

            Node_247::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_215_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_247.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_247_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_335_isNodeDirty |= g_transaction.node_247_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #248
        if (g_transaction.node_248_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(248);

            Node_248::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_217._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_248.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_248_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_270_isNodeDirty |= g_transaction.node_248_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #249
        if (g_transaction.node_249_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(249);

            Node_249::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_218_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_249.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_249_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_338_isNodeDirty |= g_transaction.node_249_isOutputDirty_OUT;
        }

    }
    { // xod__color__pulse_on_change__color #250
        if (g_transaction.node_250_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(250);

            Node_250::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_220._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_250.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_250_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_271_isNodeDirty |= g_transaction.node_250_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #251
        if (g_transaction.node_251_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(251);

            Node_251::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_221_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_251.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_251_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_341_isNodeDirty |= g_transaction.node_251_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #252
        if (g_transaction.node_252_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(252);

            Node_252::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_222._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_252.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_272_isNodeDirty = true;
        }

    }
    { // xod__core__not #253
        if (g_transaction.node_253_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(253);

            Node_253::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_223._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_253.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_273_isNodeDirty = true;
        }

    }
    { // xod__core__not #254
        if (g_transaction.node_254_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(254);

            Node_254::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_224._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_254.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_274_isNodeDirty = true;
        }

    }
    { // xod__core__not #255
        if (g_transaction.node_255_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(255);

            Node_255::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_225._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_255.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_275_isNodeDirty = true;
        }

    }
    { // xod__core__not #256
        if (g_transaction.node_256_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(256);

            Node_256::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_226._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_256.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_276_isNodeDirty = true;
        }

    }
    { // xod__core__not #257
        if (g_transaction.node_257_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(257);

            Node_257::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_227._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_257.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_277_isNodeDirty = true;
        }

    }
    { // xod__core__not #258
        if (g_transaction.node_258_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(258);

            Node_258::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_228._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_258.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_278_isNodeDirty = true;
        }

    }
    { // xod__core__not #259
        if (g_transaction.node_259_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(259);

            Node_259::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_229._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_259.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_279_isNodeDirty = true;
        }

    }
    { // xod__core__not #260
        if (g_transaction.node_260_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(260);

            Node_260::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_230._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_260.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_280_isNodeDirty = true;
        }

    }
    { // xod__core__not #261
        if (g_transaction.node_261_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(261);

            Node_261::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_231._output_SIG;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_261.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_281_isNodeDirty = true;
        }

    }
    { // xod__core__any #262
        if (g_transaction.node_262_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(262);

            Node_262::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_232_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_262.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_262_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_313_isNodeDirty |= g_transaction.node_262_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #263
        if (g_transaction.node_263_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(263);

            Node_263::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_234_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_263.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_263_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_316_isNodeDirty |= g_transaction.node_263_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #264
        if (g_transaction.node_264_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(264);

            Node_264::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_236_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_264.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_264_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_322_isNodeDirty |= g_transaction.node_264_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #265
        if (g_transaction.node_265_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(265);

            Node_265::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_238_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_265.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_265_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_325_isNodeDirty |= g_transaction.node_265_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #266
        if (g_transaction.node_266_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(266);

            Node_266::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_240_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_266.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_266_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_328_isNodeDirty |= g_transaction.node_266_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #267
        if (g_transaction.node_267_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(267);

            Node_267::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_242_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_267.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_267_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_319_isNodeDirty |= g_transaction.node_267_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #268
        if (g_transaction.node_268_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(268);

            Node_268::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_244_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_268.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_268_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_331_isNodeDirty |= g_transaction.node_268_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #269
        if (g_transaction.node_269_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(269);

            Node_269::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_246_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_269.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_269_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_334_isNodeDirty |= g_transaction.node_269_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #270
        if (g_transaction.node_270_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(270);

            Node_270::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_248_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_270.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_270_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_337_isNodeDirty |= g_transaction.node_270_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #271
        if (g_transaction.node_271_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(271);

            Node_271::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_250_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_191_isOutputDirty_BOOT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_271.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_271_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_340_isNodeDirty |= g_transaction.node_271_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #272
        if (g_transaction.node_272_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(272);

            Node_272::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_252._output_OUT;
            ctxObj._input_Ts = node_10_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_272.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_272_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_282_isNodeDirty |= g_transaction.node_272_isOutputDirty_OUT;
            g_transaction.node_283_isNodeDirty |= g_transaction.node_272_isOutputDirty_OUT;
            g_transaction.node_284_isNodeDirty |= g_transaction.node_272_isOutputDirty_OUT;
            g_transaction.node_343_isNodeDirty |= g_transaction.node_272_isOutputDirty_OUT;
            g_transaction.node_344_isNodeDirty |= g_transaction.node_272_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #273
        if (g_transaction.node_273_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(273);

            Node_273::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_253._output_OUT;
            ctxObj._input_Ts = node_12_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_273.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_273_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_285_isNodeDirty |= g_transaction.node_273_isOutputDirty_OUT;
            g_transaction.node_286_isNodeDirty |= g_transaction.node_273_isOutputDirty_OUT;
            g_transaction.node_287_isNodeDirty |= g_transaction.node_273_isOutputDirty_OUT;
            g_transaction.node_346_isNodeDirty |= g_transaction.node_273_isOutputDirty_OUT;
            g_transaction.node_347_isNodeDirty |= g_transaction.node_273_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #274
        if (g_transaction.node_274_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(274);

            Node_274::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_254._output_OUT;
            ctxObj._input_Ts = node_14_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_274.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_274_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_288_isNodeDirty |= g_transaction.node_274_isOutputDirty_OUT;
            g_transaction.node_289_isNodeDirty |= g_transaction.node_274_isOutputDirty_OUT;
            g_transaction.node_290_isNodeDirty |= g_transaction.node_274_isOutputDirty_OUT;
            g_transaction.node_349_isNodeDirty |= g_transaction.node_274_isOutputDirty_OUT;
            g_transaction.node_350_isNodeDirty |= g_transaction.node_274_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #275
        if (g_transaction.node_275_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(275);

            Node_275::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_255._output_OUT;
            ctxObj._input_Ts = node_16_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_275.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_275_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_291_isNodeDirty |= g_transaction.node_275_isOutputDirty_OUT;
            g_transaction.node_292_isNodeDirty |= g_transaction.node_275_isOutputDirty_OUT;
            g_transaction.node_293_isNodeDirty |= g_transaction.node_275_isOutputDirty_OUT;
            g_transaction.node_352_isNodeDirty |= g_transaction.node_275_isOutputDirty_OUT;
            g_transaction.node_353_isNodeDirty |= g_transaction.node_275_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #276
        if (g_transaction.node_276_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(276);

            Node_276::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_256._output_OUT;
            ctxObj._input_Ts = node_18_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_276.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_276_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_294_isNodeDirty |= g_transaction.node_276_isOutputDirty_OUT;
            g_transaction.node_295_isNodeDirty |= g_transaction.node_276_isOutputDirty_OUT;
            g_transaction.node_296_isNodeDirty |= g_transaction.node_276_isOutputDirty_OUT;
            g_transaction.node_355_isNodeDirty |= g_transaction.node_276_isOutputDirty_OUT;
            g_transaction.node_356_isNodeDirty |= g_transaction.node_276_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #277
        if (g_transaction.node_277_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(277);

            Node_277::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_257._output_OUT;
            ctxObj._input_Ts = node_20_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_277.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_277_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_297_isNodeDirty |= g_transaction.node_277_isOutputDirty_OUT;
            g_transaction.node_298_isNodeDirty |= g_transaction.node_277_isOutputDirty_OUT;
            g_transaction.node_299_isNodeDirty |= g_transaction.node_277_isOutputDirty_OUT;
            g_transaction.node_358_isNodeDirty |= g_transaction.node_277_isOutputDirty_OUT;
            g_transaction.node_359_isNodeDirty |= g_transaction.node_277_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #278
        if (g_transaction.node_278_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(278);

            Node_278::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_258._output_OUT;
            ctxObj._input_Ts = node_118_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_278.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_278_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_300_isNodeDirty |= g_transaction.node_278_isOutputDirty_OUT;
            g_transaction.node_301_isNodeDirty |= g_transaction.node_278_isOutputDirty_OUT;
            g_transaction.node_302_isNodeDirty |= g_transaction.node_278_isOutputDirty_OUT;
            g_transaction.node_361_isNodeDirty |= g_transaction.node_278_isOutputDirty_OUT;
            g_transaction.node_362_isNodeDirty |= g_transaction.node_278_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #279
        if (g_transaction.node_279_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(279);

            Node_279::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_259._output_OUT;
            ctxObj._input_Ts = node_136_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_279.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_279_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_303_isNodeDirty |= g_transaction.node_279_isOutputDirty_OUT;
            g_transaction.node_304_isNodeDirty |= g_transaction.node_279_isOutputDirty_OUT;
            g_transaction.node_305_isNodeDirty |= g_transaction.node_279_isOutputDirty_OUT;
            g_transaction.node_364_isNodeDirty |= g_transaction.node_279_isOutputDirty_OUT;
            g_transaction.node_365_isNodeDirty |= g_transaction.node_279_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #280
        if (g_transaction.node_280_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(280);

            Node_280::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_260._output_OUT;
            ctxObj._input_Ts = node_154_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_280.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_280_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_306_isNodeDirty |= g_transaction.node_280_isOutputDirty_OUT;
            g_transaction.node_307_isNodeDirty |= g_transaction.node_280_isOutputDirty_OUT;
            g_transaction.node_308_isNodeDirty |= g_transaction.node_280_isOutputDirty_OUT;
            g_transaction.node_367_isNodeDirty |= g_transaction.node_280_isOutputDirty_OUT;
            g_transaction.node_368_isNodeDirty |= g_transaction.node_280_isOutputDirty_OUT;
        }

    }
    { // xod__core__debounce__boolean #281
        if (g_transaction.node_281_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(281);

            Node_281::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_ST = node_261._output_OUT;
            ctxObj._input_Ts = node_172_output_VAL;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_281.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_281_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_309_isNodeDirty |= g_transaction.node_281_isOutputDirty_OUT;
            g_transaction.node_310_isNodeDirty |= g_transaction.node_281_isOutputDirty_OUT;
            g_transaction.node_311_isNodeDirty |= g_transaction.node_281_isOutputDirty_OUT;
            g_transaction.node_370_isNodeDirty |= g_transaction.node_281_isOutputDirty_OUT;
            g_transaction.node_371_isNodeDirty |= g_transaction.node_281_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #282
        if (g_transaction.node_282_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(282);

            Node_282::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_272._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_282.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_312_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #283
        if (g_transaction.node_283_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(283);

            Node_283::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_272._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_283.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_283_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_313_isNodeDirty |= g_transaction.node_283_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #284
        if (g_transaction.node_284_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(284);

            Node_284::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_272._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_284.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_284_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_314_isNodeDirty |= g_transaction.node_284_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #285
        if (g_transaction.node_285_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(285);

            Node_285::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_273._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_285.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_315_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #286
        if (g_transaction.node_286_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(286);

            Node_286::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_273._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_286.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_286_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_316_isNodeDirty |= g_transaction.node_286_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #287
        if (g_transaction.node_287_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(287);

            Node_287::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_273._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_287.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_287_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_317_isNodeDirty |= g_transaction.node_287_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #288
        if (g_transaction.node_288_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(288);

            Node_288::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_274._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_288.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_318_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #289
        if (g_transaction.node_289_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(289);

            Node_289::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_274._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_289.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_289_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_319_isNodeDirty |= g_transaction.node_289_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #290
        if (g_transaction.node_290_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(290);

            Node_290::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_274._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_290.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_290_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_320_isNodeDirty |= g_transaction.node_290_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #291
        if (g_transaction.node_291_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(291);

            Node_291::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_275._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_291.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_321_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #292
        if (g_transaction.node_292_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(292);

            Node_292::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_275._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_292.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_292_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_322_isNodeDirty |= g_transaction.node_292_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #293
        if (g_transaction.node_293_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(293);

            Node_293::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_275._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_293.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_293_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_323_isNodeDirty |= g_transaction.node_293_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #294
        if (g_transaction.node_294_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(294);

            Node_294::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_276._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_294.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_324_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #295
        if (g_transaction.node_295_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(295);

            Node_295::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_276._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_295.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_295_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_325_isNodeDirty |= g_transaction.node_295_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #296
        if (g_transaction.node_296_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(296);

            Node_296::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_276._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_296.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_296_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_326_isNodeDirty |= g_transaction.node_296_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #297
        if (g_transaction.node_297_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(297);

            Node_297::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_277._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_297.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_327_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #298
        if (g_transaction.node_298_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(298);

            Node_298::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_277._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_298.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_298_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_328_isNodeDirty |= g_transaction.node_298_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #299
        if (g_transaction.node_299_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(299);

            Node_299::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_277._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_299.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_299_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_329_isNodeDirty |= g_transaction.node_299_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #300
        if (g_transaction.node_300_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(300);

            Node_300::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_278._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_300.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_330_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #301
        if (g_transaction.node_301_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(301);

            Node_301::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_278._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_301.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_301_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_331_isNodeDirty |= g_transaction.node_301_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #302
        if (g_transaction.node_302_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(302);

            Node_302::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_278._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_302.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_302_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_332_isNodeDirty |= g_transaction.node_302_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #303
        if (g_transaction.node_303_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(303);

            Node_303::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_279._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_303.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_333_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #304
        if (g_transaction.node_304_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(304);

            Node_304::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_279._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_304.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_304_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_334_isNodeDirty |= g_transaction.node_304_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #305
        if (g_transaction.node_305_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(305);

            Node_305::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_279._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_305.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_305_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_335_isNodeDirty |= g_transaction.node_305_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #306
        if (g_transaction.node_306_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(306);

            Node_306::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_280._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_306.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_336_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #307
        if (g_transaction.node_307_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(307);

            Node_307::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_280._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_307.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_307_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_337_isNodeDirty |= g_transaction.node_307_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #308
        if (g_transaction.node_308_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(308);

            Node_308::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_280._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_308.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_308_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_338_isNodeDirty |= g_transaction.node_308_isOutputDirty_OUT;
        }

    }
    { // xod__core__not #309
        if (g_transaction.node_309_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(309);

            Node_309::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_281._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`

            node_309.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
            g_transaction.node_339_isNodeDirty = true;
        }

    }
    { // xod__core__cast_to_pulse__boolean #310
        if (g_transaction.node_310_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(310);

            Node_310::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_281._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_310.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_310_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_340_isNodeDirty |= g_transaction.node_310_isOutputDirty_OUT;
        }

    }
    { // xod__core__cast_to_pulse__boolean #311
        if (g_transaction.node_311_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(311);

            Node_311::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_281._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_311.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_311_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_341_isNodeDirty |= g_transaction.node_311_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #312
        if (g_transaction.node_312_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(312);

            Node_312::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_282._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_312.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_312_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_342_isNodeDirty |= g_transaction.node_312_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #313
        if (g_transaction.node_313_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(313);

            Node_313::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_262_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_283_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_313.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_313_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_343_isNodeDirty |= g_transaction.node_313_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #314
        if (g_transaction.node_314_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(314);

            Node_314::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_233_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_284_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_314.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_314_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_344_isNodeDirty |= g_transaction.node_314_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #315
        if (g_transaction.node_315_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(315);

            Node_315::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_285._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_315.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_315_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_345_isNodeDirty |= g_transaction.node_315_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #316
        if (g_transaction.node_316_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(316);

            Node_316::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_263_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_286_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_316.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_316_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_346_isNodeDirty |= g_transaction.node_316_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #317
        if (g_transaction.node_317_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(317);

            Node_317::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_235_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_287_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_317.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_317_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_347_isNodeDirty |= g_transaction.node_317_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #318
        if (g_transaction.node_318_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(318);

            Node_318::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_288._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_318.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_318_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_348_isNodeDirty |= g_transaction.node_318_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #319
        if (g_transaction.node_319_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(319);

            Node_319::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_267_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_289_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_319.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_319_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_349_isNodeDirty |= g_transaction.node_319_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #320
        if (g_transaction.node_320_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(320);

            Node_320::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_243_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_290_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_320.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_320_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_350_isNodeDirty |= g_transaction.node_320_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #321
        if (g_transaction.node_321_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(321);

            Node_321::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_291._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_321.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_321_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_351_isNodeDirty |= g_transaction.node_321_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #322
        if (g_transaction.node_322_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(322);

            Node_322::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_264_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_292_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_322.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_322_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_352_isNodeDirty |= g_transaction.node_322_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #323
        if (g_transaction.node_323_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(323);

            Node_323::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_237_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_293_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_323.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_323_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_353_isNodeDirty |= g_transaction.node_323_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #324
        if (g_transaction.node_324_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(324);

            Node_324::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_294._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_324.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_324_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_354_isNodeDirty |= g_transaction.node_324_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #325
        if (g_transaction.node_325_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(325);

            Node_325::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_265_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_295_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_325.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_325_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_355_isNodeDirty |= g_transaction.node_325_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #326
        if (g_transaction.node_326_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(326);

            Node_326::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_239_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_296_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_326.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_326_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_356_isNodeDirty |= g_transaction.node_326_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #327
        if (g_transaction.node_327_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(327);

            Node_327::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_297._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_327.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_327_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_357_isNodeDirty |= g_transaction.node_327_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #328
        if (g_transaction.node_328_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(328);

            Node_328::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_266_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_298_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_328.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_328_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_358_isNodeDirty |= g_transaction.node_328_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #329
        if (g_transaction.node_329_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(329);

            Node_329::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_241_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_299_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_329.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_329_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_359_isNodeDirty |= g_transaction.node_329_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #330
        if (g_transaction.node_330_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(330);

            Node_330::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_300._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_330.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_330_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_360_isNodeDirty |= g_transaction.node_330_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #331
        if (g_transaction.node_331_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(331);

            Node_331::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_268_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_301_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_331.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_331_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_361_isNodeDirty |= g_transaction.node_331_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #332
        if (g_transaction.node_332_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(332);

            Node_332::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_245_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_302_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_332.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_332_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_362_isNodeDirty |= g_transaction.node_332_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #333
        if (g_transaction.node_333_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(333);

            Node_333::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_303._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_333.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_333_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_363_isNodeDirty |= g_transaction.node_333_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #334
        if (g_transaction.node_334_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(334);

            Node_334::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_269_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_304_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_334.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_334_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_364_isNodeDirty |= g_transaction.node_334_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #335
        if (g_transaction.node_335_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(335);

            Node_335::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_247_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_305_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_335.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_335_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_365_isNodeDirty |= g_transaction.node_335_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #336
        if (g_transaction.node_336_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(336);

            Node_336::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_306._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_336.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_336_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_366_isNodeDirty |= g_transaction.node_336_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #337
        if (g_transaction.node_337_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(337);

            Node_337::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_270_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_307_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_337.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_337_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_367_isNodeDirty |= g_transaction.node_337_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #338
        if (g_transaction.node_338_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(338);

            Node_338::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_249_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_308_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_338.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_338_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_368_isNodeDirty |= g_transaction.node_338_isOutputDirty_OUT;
        }

    }
    { // xod__core__pulse_on_true #339
        if (g_transaction.node_339_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(339);

            Node_339::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_IN = node_309._output_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_339.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_339_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_369_isNodeDirty |= g_transaction.node_339_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #340
        if (g_transaction.node_340_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(340);

            Node_340::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_271_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_310_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_340.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_340_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_370_isNodeDirty |= g_transaction.node_340_isOutputDirty_OUT;
        }

    }
    { // xod__core__any #341
        if (g_transaction.node_341_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(341);

            Node_341::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_251_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_311_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_341.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_341_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_371_isNodeDirty |= g_transaction.node_341_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #342
        if (g_transaction.node_342_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(342);

            Node_342::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_25_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_312_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_342.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_342_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_372_isNodeDirty |= g_transaction.node_342_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #343
        if (g_transaction.node_343_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(343);

            Node_343::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_272._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_313_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_343.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_343_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_373_isNodeDirty |= g_transaction.node_343_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #344
        if (g_transaction.node_344_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(344);

            Node_344::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_272._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_314_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_344.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_344_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_373_isNodeDirty |= g_transaction.node_344_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #345
        if (g_transaction.node_345_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(345);

            Node_345::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_36_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_315_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_345.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_345_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_374_isNodeDirty |= g_transaction.node_345_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #346
        if (g_transaction.node_346_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(346);

            Node_346::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_273._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_316_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_346.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_346_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_375_isNodeDirty |= g_transaction.node_346_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #347
        if (g_transaction.node_347_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(347);

            Node_347::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_273._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_317_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_347.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_347_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_375_isNodeDirty |= g_transaction.node_347_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #348
        if (g_transaction.node_348_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(348);

            Node_348::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_47_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_318_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_348.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_348_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_376_isNodeDirty |= g_transaction.node_348_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #349
        if (g_transaction.node_349_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(349);

            Node_349::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_274._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_319_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_349.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_349_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_377_isNodeDirty |= g_transaction.node_349_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #350
        if (g_transaction.node_350_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(350);

            Node_350::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_274._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_320_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_350.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_350_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_377_isNodeDirty |= g_transaction.node_350_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #351
        if (g_transaction.node_351_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(351);

            Node_351::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_58_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_321_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_351.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_351_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_378_isNodeDirty |= g_transaction.node_351_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #352
        if (g_transaction.node_352_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(352);

            Node_352::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_275._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_322_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_352.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_352_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_379_isNodeDirty |= g_transaction.node_352_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #353
        if (g_transaction.node_353_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(353);

            Node_353::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_275._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_323_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_353.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_353_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_379_isNodeDirty |= g_transaction.node_353_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #354
        if (g_transaction.node_354_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(354);

            Node_354::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_69_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_324_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_354.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_354_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_380_isNodeDirty |= g_transaction.node_354_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #355
        if (g_transaction.node_355_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(355);

            Node_355::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_276._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_325_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_355.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_355_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_381_isNodeDirty |= g_transaction.node_355_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #356
        if (g_transaction.node_356_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(356);

            Node_356::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_276._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_326_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_356.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_356_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_381_isNodeDirty |= g_transaction.node_356_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #357
        if (g_transaction.node_357_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(357);

            Node_357::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_80_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_327_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_357.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_357_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_382_isNodeDirty |= g_transaction.node_357_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #358
        if (g_transaction.node_358_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(358);

            Node_358::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_277._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_328_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_358.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_358_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_383_isNodeDirty |= g_transaction.node_358_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #359
        if (g_transaction.node_359_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(359);

            Node_359::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_277._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_329_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_359.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_359_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_383_isNodeDirty |= g_transaction.node_359_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #360
        if (g_transaction.node_360_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(360);

            Node_360::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_123_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_330_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_360.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_360_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_384_isNodeDirty |= g_transaction.node_360_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #361
        if (g_transaction.node_361_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(361);

            Node_361::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_278._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_331_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_361.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_361_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_385_isNodeDirty |= g_transaction.node_361_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #362
        if (g_transaction.node_362_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(362);

            Node_362::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_278._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_332_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_362.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_362_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_385_isNodeDirty |= g_transaction.node_362_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #363
        if (g_transaction.node_363_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(363);

            Node_363::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_141_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_333_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_363.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_363_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_386_isNodeDirty |= g_transaction.node_363_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #364
        if (g_transaction.node_364_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(364);

            Node_364::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_279._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_334_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_364.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_364_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_387_isNodeDirty |= g_transaction.node_364_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #365
        if (g_transaction.node_365_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(365);

            Node_365::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_279._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_335_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_365.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_365_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_387_isNodeDirty |= g_transaction.node_365_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #366
        if (g_transaction.node_366_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(366);

            Node_366::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_159_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_336_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_366.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_366_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_388_isNodeDirty |= g_transaction.node_366_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #367
        if (g_transaction.node_367_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(367);

            Node_367::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_280._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_337_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_367.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_367_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_389_isNodeDirty |= g_transaction.node_367_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #368
        if (g_transaction.node_368_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(368);

            Node_368::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_280._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_338_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_368.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_368_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_389_isNodeDirty |= g_transaction.node_368_isOutputDirty_OUT;
        }

    }
    { // xod__uart__soft_uart #369
        if (g_transaction.node_369_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(369);

            Node_369::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_BAUD = node_177_output_VAL;

            ctxObj._isInputDirty_INIT = g_transaction.node_339_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_UART = false;
            ctxObj._isOutputDirty_DONE = false;

            node_369.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_369_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_390_isNodeDirty |= g_transaction.node_369_isOutputDirty_DONE;
        }

    }
    { // xod__core__gate__pulse #370
        if (g_transaction.node_370_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(370);

            Node_370::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_281._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_340_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_370.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_370_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_391_isNodeDirty |= g_transaction.node_370_isOutputDirty_OUT;
        }

    }
    { // xod__core__gate__pulse #371
        if (g_transaction.node_371_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(371);

            Node_371::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_EN = node_281._output_OUT;

            ctxObj._isInputDirty_IN = g_transaction.node_341_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_371.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_371_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_391_isNodeDirty |= g_transaction.node_371_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #372
        if (g_transaction.node_372_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(372);

            Node_372::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_26_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_342_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_372::NodeErrors previousErrors = node_372.errors;

            node_372.errors.output_DONE = false;

            node_372.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_372_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_372.errors.flags) {
                detail::printErrorToDebugSerial(372, node_372.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_372.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_372.errors.output_DONE) {
                    g_transaction.node_392_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_392_isNodeDirty |= g_transaction.node_372_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_372.errors.flags) {
            if (node_372.errors.output_DONE) {
                g_transaction.node_392_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #373
        if (g_transaction.node_373_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(373);

            Node_373::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_344_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_343_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_373.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_373_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_393_isNodeDirty |= g_transaction.node_373_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #374
        if (g_transaction.node_374_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(374);

            Node_374::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_37_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_345_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_374::NodeErrors previousErrors = node_374.errors;

            node_374.errors.output_DONE = false;

            node_374.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_374_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_374.errors.flags) {
                detail::printErrorToDebugSerial(374, node_374.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_374.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_374.errors.output_DONE) {
                    g_transaction.node_394_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_394_isNodeDirty |= g_transaction.node_374_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_374.errors.flags) {
            if (node_374.errors.output_DONE) {
                g_transaction.node_394_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #375
        if (g_transaction.node_375_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(375);

            Node_375::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_347_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_346_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_375.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_375_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_395_isNodeDirty |= g_transaction.node_375_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #376
        if (g_transaction.node_376_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(376);

            Node_376::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_48_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_348_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_376::NodeErrors previousErrors = node_376.errors;

            node_376.errors.output_DONE = false;

            node_376.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_376_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_376.errors.flags) {
                detail::printErrorToDebugSerial(376, node_376.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_376.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_376.errors.output_DONE) {
                    g_transaction.node_396_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_396_isNodeDirty |= g_transaction.node_376_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_376.errors.flags) {
            if (node_376.errors.output_DONE) {
                g_transaction.node_396_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #377
        if (g_transaction.node_377_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(377);

            Node_377::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_350_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_349_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_377.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_377_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_397_isNodeDirty |= g_transaction.node_377_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #378
        if (g_transaction.node_378_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(378);

            Node_378::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_59_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_351_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_378::NodeErrors previousErrors = node_378.errors;

            node_378.errors.output_DONE = false;

            node_378.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_378_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_378.errors.flags) {
                detail::printErrorToDebugSerial(378, node_378.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_378.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_378.errors.output_DONE) {
                    g_transaction.node_398_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_398_isNodeDirty |= g_transaction.node_378_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_378.errors.flags) {
            if (node_378.errors.output_DONE) {
                g_transaction.node_398_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #379
        if (g_transaction.node_379_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(379);

            Node_379::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_353_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_352_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_379.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_379_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_399_isNodeDirty |= g_transaction.node_379_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #380
        if (g_transaction.node_380_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(380);

            Node_380::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_70_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_354_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_380::NodeErrors previousErrors = node_380.errors;

            node_380.errors.output_DONE = false;

            node_380.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_380_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_380.errors.flags) {
                detail::printErrorToDebugSerial(380, node_380.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_380.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_380.errors.output_DONE) {
                    g_transaction.node_400_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_400_isNodeDirty |= g_transaction.node_380_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_380.errors.flags) {
            if (node_380.errors.output_DONE) {
                g_transaction.node_400_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #381
        if (g_transaction.node_381_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(381);

            Node_381::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_356_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_355_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_381.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_381_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_401_isNodeDirty |= g_transaction.node_381_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #382
        if (g_transaction.node_382_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(382);

            Node_382::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_81_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_357_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_382::NodeErrors previousErrors = node_382.errors;

            node_382.errors.output_DONE = false;

            node_382.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_382_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_382.errors.flags) {
                detail::printErrorToDebugSerial(382, node_382.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_382.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_382.errors.output_DONE) {
                    g_transaction.node_402_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_402_isNodeDirty |= g_transaction.node_382_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_382.errors.flags) {
            if (node_382.errors.output_DONE) {
                g_transaction.node_402_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #383
        if (g_transaction.node_383_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(383);

            Node_383::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_359_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_358_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_383.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_383_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_403_isNodeDirty |= g_transaction.node_383_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #384
        if (g_transaction.node_384_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(384);

            Node_384::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_124_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_360_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_384::NodeErrors previousErrors = node_384.errors;

            node_384.errors.output_DONE = false;

            node_384.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_384_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_384.errors.flags) {
                detail::printErrorToDebugSerial(384, node_384.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_384.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_384.errors.output_DONE) {
                    g_transaction.node_404_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_404_isNodeDirty |= g_transaction.node_384_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_384.errors.flags) {
            if (node_384.errors.output_DONE) {
                g_transaction.node_404_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #385
        if (g_transaction.node_385_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(385);

            Node_385::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_362_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_361_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_385.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_385_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_405_isNodeDirty |= g_transaction.node_385_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #386
        if (g_transaction.node_386_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(386);

            Node_386::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_142_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_363_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_386::NodeErrors previousErrors = node_386.errors;

            node_386.errors.output_DONE = false;

            node_386.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_386_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_386.errors.flags) {
                detail::printErrorToDebugSerial(386, node_386.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_386.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_386.errors.output_DONE) {
                    g_transaction.node_406_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_406_isNodeDirty |= g_transaction.node_386_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_386.errors.flags) {
            if (node_386.errors.output_DONE) {
                g_transaction.node_406_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #387
        if (g_transaction.node_387_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(387);

            Node_387::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_365_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_364_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_387.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_387_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_407_isNodeDirty |= g_transaction.node_387_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #388
        if (g_transaction.node_388_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(388);

            Node_388::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_160_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_366_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_388::NodeErrors previousErrors = node_388.errors;

            node_388.errors.output_DONE = false;

            node_388.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_388_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_388.errors.flags) {
                detail::printErrorToDebugSerial(388, node_388.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_388.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_388.errors.output_DONE) {
                    g_transaction.node_408_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_408_isNodeDirty |= g_transaction.node_388_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_388.errors.flags) {
            if (node_388.errors.output_DONE) {
                g_transaction.node_408_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #389
        if (g_transaction.node_389_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(389);

            Node_389::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_368_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_367_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_389.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_389_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_409_isNodeDirty |= g_transaction.node_389_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #390
        if (g_transaction.node_390_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(390);

            Node_390::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_178_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_369_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_390::NodeErrors previousErrors = node_390.errors;

            node_390.errors.output_DONE = false;

            node_390.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_390_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_390.errors.flags) {
                detail::printErrorToDebugSerial(390, node_390.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_390.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_390.errors.output_DONE) {
                    g_transaction.node_410_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_410_isNodeDirty |= g_transaction.node_390_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_390.errors.flags) {
            if (node_390.errors.output_DONE) {
                g_transaction.node_410_hasUpstreamError = true;
            }
        }
    }
    { // xod__core__any #391
        if (g_transaction.node_391_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(391);

            Node_391::ContextObject ctxObj;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN1 = g_transaction.node_371_isOutputDirty_OUT;
            ctxObj._isInputDirty_IN2 = g_transaction.node_370_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            node_391.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_391_isOutputDirty_OUT = ctxObj._isOutputDirty_OUT;

            // mark downstream nodes dirty
            g_transaction.node_411_isNodeDirty |= g_transaction.node_391_isOutputDirty_OUT;
        }

    }
    { // xod__uart__write_byte #392

        if (g_transaction.node_392_hasUpstreamError) {
            g_transaction.node_412_hasUpstreamError = true;
        } else if (g_transaction.node_392_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(392);

            Node_392::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_28_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_372_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_392::NodeErrors previousErrors = node_392.errors;

            node_392.errors.output_DONE = false;

            node_392.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_392_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_392.errors.flags) {
                detail::printErrorToDebugSerial(392, node_392.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_392.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_392.errors.output_DONE) {
                    g_transaction.node_412_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_412_isNodeDirty |= g_transaction.node_392_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_392.errors.flags) {
            if (node_392.errors.output_DONE) {
                g_transaction.node_412_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #393
        if (g_transaction.node_393_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(393);

            Node_393::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_192._output_DEV;
            ctxObj._input_C = node_193._output_OUT;
            ctxObj._input_NUM = node_91_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_373_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_393.emitValue<Node_393::output_DEVU0027>(&ctxObj, node_393.getValue<Node_393::input_DEV>(&ctxObj));
            }
            node_393.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #394

        if (g_transaction.node_394_hasUpstreamError) {
            g_transaction.node_413_hasUpstreamError = true;
        } else if (g_transaction.node_394_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(394);

            Node_394::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_39_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_374_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_394::NodeErrors previousErrors = node_394.errors;

            node_394.errors.output_DONE = false;

            node_394.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_394_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_394.errors.flags) {
                detail::printErrorToDebugSerial(394, node_394.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_394.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_394.errors.output_DONE) {
                    g_transaction.node_413_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_413_isNodeDirty |= g_transaction.node_394_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_394.errors.flags) {
            if (node_394.errors.output_DONE) {
                g_transaction.node_413_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #395
        if (g_transaction.node_395_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(395);

            Node_395::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_195._output_DEV;
            ctxObj._input_C = node_196._output_OUT;
            ctxObj._input_NUM = node_96_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_375_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_395.emitValue<Node_395::output_DEVU0027>(&ctxObj, node_395.getValue<Node_395::input_DEV>(&ctxObj));
            }
            node_395.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #396

        if (g_transaction.node_396_hasUpstreamError) {
            g_transaction.node_414_hasUpstreamError = true;
        } else if (g_transaction.node_396_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(396);

            Node_396::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_50_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_376_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_396::NodeErrors previousErrors = node_396.errors;

            node_396.errors.output_DONE = false;

            node_396.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_396_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_396.errors.flags) {
                detail::printErrorToDebugSerial(396, node_396.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_396.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_396.errors.output_DONE) {
                    g_transaction.node_414_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_414_isNodeDirty |= g_transaction.node_396_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_396.errors.flags) {
            if (node_396.errors.output_DONE) {
                g_transaction.node_414_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #397
        if (g_transaction.node_397_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(397);

            Node_397::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_207._output_DEV;
            ctxObj._input_C = node_208._output_OUT;
            ctxObj._input_NUM = node_116_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_377_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_397.emitValue<Node_397::output_DEVU0027>(&ctxObj, node_397.getValue<Node_397::input_DEV>(&ctxObj));
            }
            node_397.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #398

        if (g_transaction.node_398_hasUpstreamError) {
            g_transaction.node_415_hasUpstreamError = true;
        } else if (g_transaction.node_398_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(398);

            Node_398::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_61_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_378_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_398::NodeErrors previousErrors = node_398.errors;

            node_398.errors.output_DONE = false;

            node_398.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_398_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_398.errors.flags) {
                detail::printErrorToDebugSerial(398, node_398.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_398.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_398.errors.output_DONE) {
                    g_transaction.node_415_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_415_isNodeDirty |= g_transaction.node_398_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_398.errors.flags) {
            if (node_398.errors.output_DONE) {
                g_transaction.node_415_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #399
        if (g_transaction.node_399_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(399);

            Node_399::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_198._output_DEV;
            ctxObj._input_C = node_199._output_OUT;
            ctxObj._input_NUM = node_101_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_379_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_399.emitValue<Node_399::output_DEVU0027>(&ctxObj, node_399.getValue<Node_399::input_DEV>(&ctxObj));
            }
            node_399.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #400

        if (g_transaction.node_400_hasUpstreamError) {
            g_transaction.node_416_hasUpstreamError = true;
        } else if (g_transaction.node_400_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(400);

            Node_400::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_72_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_380_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_400::NodeErrors previousErrors = node_400.errors;

            node_400.errors.output_DONE = false;

            node_400.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_400_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_400.errors.flags) {
                detail::printErrorToDebugSerial(400, node_400.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_400.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_400.errors.output_DONE) {
                    g_transaction.node_416_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_416_isNodeDirty |= g_transaction.node_400_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_400.errors.flags) {
            if (node_400.errors.output_DONE) {
                g_transaction.node_416_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #401
        if (g_transaction.node_401_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(401);

            Node_401::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_201._output_DEV;
            ctxObj._input_C = node_202._output_OUT;
            ctxObj._input_NUM = node_106_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_381_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_401.emitValue<Node_401::output_DEVU0027>(&ctxObj, node_401.getValue<Node_401::input_DEV>(&ctxObj));
            }
            node_401.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #402

        if (g_transaction.node_402_hasUpstreamError) {
            g_transaction.node_417_hasUpstreamError = true;
        } else if (g_transaction.node_402_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(402);

            Node_402::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_83_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_382_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_402::NodeErrors previousErrors = node_402.errors;

            node_402.errors.output_DONE = false;

            node_402.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_402_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_402.errors.flags) {
                detail::printErrorToDebugSerial(402, node_402.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_402.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_402.errors.output_DONE) {
                    g_transaction.node_417_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_417_isNodeDirty |= g_transaction.node_402_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_402.errors.flags) {
            if (node_402.errors.output_DONE) {
                g_transaction.node_417_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #403
        if (g_transaction.node_403_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(403);

            Node_403::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_204._output_DEV;
            ctxObj._input_C = node_205._output_OUT;
            ctxObj._input_NUM = node_111_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_383_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_403.emitValue<Node_403::output_DEVU0027>(&ctxObj, node_403.getValue<Node_403::input_DEV>(&ctxObj));
            }
            node_403.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #404

        if (g_transaction.node_404_hasUpstreamError) {
            g_transaction.node_418_hasUpstreamError = true;
        } else if (g_transaction.node_404_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(404);

            Node_404::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_126_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_384_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_404::NodeErrors previousErrors = node_404.errors;

            node_404.errors.output_DONE = false;

            node_404.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_404_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_404.errors.flags) {
                detail::printErrorToDebugSerial(404, node_404.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_404.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_404.errors.output_DONE) {
                    g_transaction.node_418_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_418_isNodeDirty |= g_transaction.node_404_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_404.errors.flags) {
            if (node_404.errors.output_DONE) {
                g_transaction.node_418_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #405
        if (g_transaction.node_405_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(405);

            Node_405::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_210._output_DEV;
            ctxObj._input_C = node_211._output_OUT;
            ctxObj._input_NUM = node_134_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_385_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_405.emitValue<Node_405::output_DEVU0027>(&ctxObj, node_405.getValue<Node_405::input_DEV>(&ctxObj));
            }
            node_405.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #406

        if (g_transaction.node_406_hasUpstreamError) {
            g_transaction.node_419_hasUpstreamError = true;
        } else if (g_transaction.node_406_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(406);

            Node_406::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_144_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_386_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_406::NodeErrors previousErrors = node_406.errors;

            node_406.errors.output_DONE = false;

            node_406.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_406_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_406.errors.flags) {
                detail::printErrorToDebugSerial(406, node_406.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_406.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_406.errors.output_DONE) {
                    g_transaction.node_419_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_419_isNodeDirty |= g_transaction.node_406_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_406.errors.flags) {
            if (node_406.errors.output_DONE) {
                g_transaction.node_419_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #407
        if (g_transaction.node_407_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(407);

            Node_407::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_213._output_DEV;
            ctxObj._input_C = node_214._output_OUT;
            ctxObj._input_NUM = node_152_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_387_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_407.emitValue<Node_407::output_DEVU0027>(&ctxObj, node_407.getValue<Node_407::input_DEV>(&ctxObj));
            }
            node_407.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #408

        if (g_transaction.node_408_hasUpstreamError) {
            g_transaction.node_420_hasUpstreamError = true;
        } else if (g_transaction.node_408_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(408);

            Node_408::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_162_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_388_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_408::NodeErrors previousErrors = node_408.errors;

            node_408.errors.output_DONE = false;

            node_408.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_408_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_408.errors.flags) {
                detail::printErrorToDebugSerial(408, node_408.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_408.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_408.errors.output_DONE) {
                    g_transaction.node_420_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_420_isNodeDirty |= g_transaction.node_408_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_408.errors.flags) {
            if (node_408.errors.output_DONE) {
                g_transaction.node_420_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #409
        if (g_transaction.node_409_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(409);

            Node_409::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_216._output_DEV;
            ctxObj._input_C = node_217._output_OUT;
            ctxObj._input_NUM = node_170_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_389_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_409.emitValue<Node_409::output_DEVU0027>(&ctxObj, node_409.getValue<Node_409::input_DEV>(&ctxObj));
            }
            node_409.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #410

        if (g_transaction.node_410_hasUpstreamError) {
            g_transaction.node_421_hasUpstreamError = true;
        } else if (g_transaction.node_410_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(410);

            Node_410::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_180_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_390_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_410::NodeErrors previousErrors = node_410.errors;

            node_410.errors.output_DONE = false;

            node_410.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_410_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_410.errors.flags) {
                detail::printErrorToDebugSerial(410, node_410.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_410.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_410.errors.output_DONE) {
                    g_transaction.node_421_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_421_isNodeDirty |= g_transaction.node_410_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_410.errors.flags) {
            if (node_410.errors.output_DONE) {
                g_transaction.node_421_hasUpstreamError = true;
            }
        }
    }
    { // xod_dev__ws2812__fill_solid #411
        if (g_transaction.node_411_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(411);

            Node_411::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_DEV = node_219._output_DEV;
            ctxObj._input_C = node_220._output_OUT;
            ctxObj._input_NUM = node_188_output_VAL;

            ctxObj._isInputDirty_DO = g_transaction.node_391_isOutputDirty_OUT;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DEVU0027 = false;
            ctxObj._isOutputDirty_DONE = false;

            if (isSettingUp()) {
                node_411.emitValue<Node_411::output_DEVU0027>(&ctxObj, node_411.getValue<Node_411::input_DEV>(&ctxObj));
            }
            node_411.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            // mark downstream nodes dirty
        }

    }
    { // xod__uart__write_byte #412

        if (g_transaction.node_412_hasUpstreamError) {
            g_transaction.node_422_hasUpstreamError = true;
        } else if (g_transaction.node_412_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(412);

            Node_412::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_22_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_392_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_412::NodeErrors previousErrors = node_412.errors;

            node_412.errors.output_DONE = false;

            node_412.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_412_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_412.errors.flags) {
                detail::printErrorToDebugSerial(412, node_412.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_412.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_412.errors.output_DONE) {
                    g_transaction.node_422_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_422_isNodeDirty |= g_transaction.node_412_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_412.errors.flags) {
            if (node_412.errors.output_DONE) {
                g_transaction.node_422_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #413

        if (g_transaction.node_413_hasUpstreamError) {
            g_transaction.node_423_hasUpstreamError = true;
        } else if (g_transaction.node_413_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(413);

            Node_413::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_33_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_394_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_413::NodeErrors previousErrors = node_413.errors;

            node_413.errors.output_DONE = false;

            node_413.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_413_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_413.errors.flags) {
                detail::printErrorToDebugSerial(413, node_413.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_413.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_413.errors.output_DONE) {
                    g_transaction.node_423_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_423_isNodeDirty |= g_transaction.node_413_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_413.errors.flags) {
            if (node_413.errors.output_DONE) {
                g_transaction.node_423_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #414

        if (g_transaction.node_414_hasUpstreamError) {
            g_transaction.node_424_hasUpstreamError = true;
        } else if (g_transaction.node_414_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(414);

            Node_414::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_44_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_396_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_414::NodeErrors previousErrors = node_414.errors;

            node_414.errors.output_DONE = false;

            node_414.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_414_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_414.errors.flags) {
                detail::printErrorToDebugSerial(414, node_414.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_414.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_414.errors.output_DONE) {
                    g_transaction.node_424_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_424_isNodeDirty |= g_transaction.node_414_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_414.errors.flags) {
            if (node_414.errors.output_DONE) {
                g_transaction.node_424_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #415

        if (g_transaction.node_415_hasUpstreamError) {
            g_transaction.node_425_hasUpstreamError = true;
        } else if (g_transaction.node_415_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(415);

            Node_415::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_55_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_398_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_415::NodeErrors previousErrors = node_415.errors;

            node_415.errors.output_DONE = false;

            node_415.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_415_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_415.errors.flags) {
                detail::printErrorToDebugSerial(415, node_415.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_415.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_415.errors.output_DONE) {
                    g_transaction.node_425_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_425_isNodeDirty |= g_transaction.node_415_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_415.errors.flags) {
            if (node_415.errors.output_DONE) {
                g_transaction.node_425_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #416

        if (g_transaction.node_416_hasUpstreamError) {
            g_transaction.node_426_hasUpstreamError = true;
        } else if (g_transaction.node_416_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(416);

            Node_416::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_66_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_400_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_416::NodeErrors previousErrors = node_416.errors;

            node_416.errors.output_DONE = false;

            node_416.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_416_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_416.errors.flags) {
                detail::printErrorToDebugSerial(416, node_416.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_416.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_416.errors.output_DONE) {
                    g_transaction.node_426_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_426_isNodeDirty |= g_transaction.node_416_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_416.errors.flags) {
            if (node_416.errors.output_DONE) {
                g_transaction.node_426_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #417

        if (g_transaction.node_417_hasUpstreamError) {
            g_transaction.node_427_hasUpstreamError = true;
        } else if (g_transaction.node_417_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(417);

            Node_417::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_77_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_402_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_417::NodeErrors previousErrors = node_417.errors;

            node_417.errors.output_DONE = false;

            node_417.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_417_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_417.errors.flags) {
                detail::printErrorToDebugSerial(417, node_417.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_417.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_417.errors.output_DONE) {
                    g_transaction.node_427_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_427_isNodeDirty |= g_transaction.node_417_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_417.errors.flags) {
            if (node_417.errors.output_DONE) {
                g_transaction.node_427_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #418

        if (g_transaction.node_418_hasUpstreamError) {
            g_transaction.node_428_hasUpstreamError = true;
        } else if (g_transaction.node_418_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(418);

            Node_418::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_120_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_404_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_418::NodeErrors previousErrors = node_418.errors;

            node_418.errors.output_DONE = false;

            node_418.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_418_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_418.errors.flags) {
                detail::printErrorToDebugSerial(418, node_418.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_418.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_418.errors.output_DONE) {
                    g_transaction.node_428_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_428_isNodeDirty |= g_transaction.node_418_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_418.errors.flags) {
            if (node_418.errors.output_DONE) {
                g_transaction.node_428_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #419

        if (g_transaction.node_419_hasUpstreamError) {
            g_transaction.node_429_hasUpstreamError = true;
        } else if (g_transaction.node_419_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(419);

            Node_419::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_138_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_406_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_419::NodeErrors previousErrors = node_419.errors;

            node_419.errors.output_DONE = false;

            node_419.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_419_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_419.errors.flags) {
                detail::printErrorToDebugSerial(419, node_419.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_419.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_419.errors.output_DONE) {
                    g_transaction.node_429_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_429_isNodeDirty |= g_transaction.node_419_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_419.errors.flags) {
            if (node_419.errors.output_DONE) {
                g_transaction.node_429_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #420

        if (g_transaction.node_420_hasUpstreamError) {
            g_transaction.node_430_hasUpstreamError = true;
        } else if (g_transaction.node_420_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(420);

            Node_420::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_156_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_408_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_420::NodeErrors previousErrors = node_420.errors;

            node_420.errors.output_DONE = false;

            node_420.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_420_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_420.errors.flags) {
                detail::printErrorToDebugSerial(420, node_420.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_420.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_420.errors.output_DONE) {
                    g_transaction.node_430_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_430_isNodeDirty |= g_transaction.node_420_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_420.errors.flags) {
            if (node_420.errors.output_DONE) {
                g_transaction.node_430_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #421

        if (g_transaction.node_421_hasUpstreamError) {
            g_transaction.node_431_hasUpstreamError = true;
        } else if (g_transaction.node_421_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(421);

            Node_421::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_174_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_410_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_421::NodeErrors previousErrors = node_421.errors;

            node_421.errors.output_DONE = false;

            node_421.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_421_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_421.errors.flags) {
                detail::printErrorToDebugSerial(421, node_421.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_421.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_421.errors.output_DONE) {
                    g_transaction.node_431_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_431_isNodeDirty |= g_transaction.node_421_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_421.errors.flags) {
            if (node_421.errors.output_DONE) {
                g_transaction.node_431_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #422

        if (g_transaction.node_422_hasUpstreamError) {
            g_transaction.node_432_hasUpstreamError = true;
        } else if (g_transaction.node_422_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(422);

            Node_422::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_30_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_412_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_422::NodeErrors previousErrors = node_422.errors;

            node_422.errors.output_DONE = false;

            node_422.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_422_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_422.errors.flags) {
                detail::printErrorToDebugSerial(422, node_422.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_422.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_422.errors.output_DONE) {
                    g_transaction.node_432_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_432_isNodeDirty |= g_transaction.node_422_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_422.errors.flags) {
            if (node_422.errors.output_DONE) {
                g_transaction.node_432_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #423

        if (g_transaction.node_423_hasUpstreamError) {
            g_transaction.node_433_hasUpstreamError = true;
        } else if (g_transaction.node_423_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(423);

            Node_423::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_41_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_413_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_423::NodeErrors previousErrors = node_423.errors;

            node_423.errors.output_DONE = false;

            node_423.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_423_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_423.errors.flags) {
                detail::printErrorToDebugSerial(423, node_423.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_423.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_423.errors.output_DONE) {
                    g_transaction.node_433_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_433_isNodeDirty |= g_transaction.node_423_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_423.errors.flags) {
            if (node_423.errors.output_DONE) {
                g_transaction.node_433_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #424

        if (g_transaction.node_424_hasUpstreamError) {
            g_transaction.node_434_hasUpstreamError = true;
        } else if (g_transaction.node_424_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(424);

            Node_424::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_52_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_414_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_424::NodeErrors previousErrors = node_424.errors;

            node_424.errors.output_DONE = false;

            node_424.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_424_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_424.errors.flags) {
                detail::printErrorToDebugSerial(424, node_424.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_424.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_424.errors.output_DONE) {
                    g_transaction.node_434_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_434_isNodeDirty |= g_transaction.node_424_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_424.errors.flags) {
            if (node_424.errors.output_DONE) {
                g_transaction.node_434_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #425

        if (g_transaction.node_425_hasUpstreamError) {
            g_transaction.node_435_hasUpstreamError = true;
        } else if (g_transaction.node_425_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(425);

            Node_425::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_63_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_415_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_425::NodeErrors previousErrors = node_425.errors;

            node_425.errors.output_DONE = false;

            node_425.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_425_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_425.errors.flags) {
                detail::printErrorToDebugSerial(425, node_425.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_425.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_425.errors.output_DONE) {
                    g_transaction.node_435_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_435_isNodeDirty |= g_transaction.node_425_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_425.errors.flags) {
            if (node_425.errors.output_DONE) {
                g_transaction.node_435_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #426

        if (g_transaction.node_426_hasUpstreamError) {
            g_transaction.node_436_hasUpstreamError = true;
        } else if (g_transaction.node_426_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(426);

            Node_426::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_74_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_416_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_426::NodeErrors previousErrors = node_426.errors;

            node_426.errors.output_DONE = false;

            node_426.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_426_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_426.errors.flags) {
                detail::printErrorToDebugSerial(426, node_426.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_426.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_426.errors.output_DONE) {
                    g_transaction.node_436_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_436_isNodeDirty |= g_transaction.node_426_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_426.errors.flags) {
            if (node_426.errors.output_DONE) {
                g_transaction.node_436_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #427

        if (g_transaction.node_427_hasUpstreamError) {
            g_transaction.node_437_hasUpstreamError = true;
        } else if (g_transaction.node_427_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(427);

            Node_427::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_85_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_417_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_427::NodeErrors previousErrors = node_427.errors;

            node_427.errors.output_DONE = false;

            node_427.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_427_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_427.errors.flags) {
                detail::printErrorToDebugSerial(427, node_427.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_427.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_427.errors.output_DONE) {
                    g_transaction.node_437_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_437_isNodeDirty |= g_transaction.node_427_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_427.errors.flags) {
            if (node_427.errors.output_DONE) {
                g_transaction.node_437_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #428

        if (g_transaction.node_428_hasUpstreamError) {
            g_transaction.node_438_hasUpstreamError = true;
        } else if (g_transaction.node_428_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(428);

            Node_428::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_128_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_418_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_428::NodeErrors previousErrors = node_428.errors;

            node_428.errors.output_DONE = false;

            node_428.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_428_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_428.errors.flags) {
                detail::printErrorToDebugSerial(428, node_428.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_428.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_428.errors.output_DONE) {
                    g_transaction.node_438_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_438_isNodeDirty |= g_transaction.node_428_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_428.errors.flags) {
            if (node_428.errors.output_DONE) {
                g_transaction.node_438_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #429

        if (g_transaction.node_429_hasUpstreamError) {
            g_transaction.node_439_hasUpstreamError = true;
        } else if (g_transaction.node_429_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(429);

            Node_429::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_146_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_419_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_429::NodeErrors previousErrors = node_429.errors;

            node_429.errors.output_DONE = false;

            node_429.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_429_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_429.errors.flags) {
                detail::printErrorToDebugSerial(429, node_429.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_429.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_429.errors.output_DONE) {
                    g_transaction.node_439_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_439_isNodeDirty |= g_transaction.node_429_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_429.errors.flags) {
            if (node_429.errors.output_DONE) {
                g_transaction.node_439_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #430

        if (g_transaction.node_430_hasUpstreamError) {
            g_transaction.node_440_hasUpstreamError = true;
        } else if (g_transaction.node_430_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(430);

            Node_430::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_164_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_420_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_430::NodeErrors previousErrors = node_430.errors;

            node_430.errors.output_DONE = false;

            node_430.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_430_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_430.errors.flags) {
                detail::printErrorToDebugSerial(430, node_430.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_430.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_430.errors.output_DONE) {
                    g_transaction.node_440_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_440_isNodeDirty |= g_transaction.node_430_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_430.errors.flags) {
            if (node_430.errors.output_DONE) {
                g_transaction.node_440_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #431

        if (g_transaction.node_431_hasUpstreamError) {
            g_transaction.node_441_hasUpstreamError = true;
        } else if (g_transaction.node_431_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(431);

            Node_431::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_182_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_421_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_431::NodeErrors previousErrors = node_431.errors;

            node_431.errors.output_DONE = false;

            node_431.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_431_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_431.errors.flags) {
                detail::printErrorToDebugSerial(431, node_431.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_431.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_431.errors.output_DONE) {
                    g_transaction.node_441_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_441_isNodeDirty |= g_transaction.node_431_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_431.errors.flags) {
            if (node_431.errors.output_DONE) {
                g_transaction.node_441_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #432

        if (g_transaction.node_432_hasUpstreamError) {
            g_transaction.node_442_hasUpstreamError = true;
        } else if (g_transaction.node_432_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(432);

            Node_432::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_29_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_422_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_432::NodeErrors previousErrors = node_432.errors;

            node_432.errors.output_DONE = false;

            node_432.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_432_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_432.errors.flags) {
                detail::printErrorToDebugSerial(432, node_432.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_432.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_432.errors.output_DONE) {
                    g_transaction.node_442_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_442_isNodeDirty |= g_transaction.node_432_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_432.errors.flags) {
            if (node_432.errors.output_DONE) {
                g_transaction.node_442_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #433

        if (g_transaction.node_433_hasUpstreamError) {
            g_transaction.node_443_hasUpstreamError = true;
        } else if (g_transaction.node_433_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(433);

            Node_433::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_40_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_423_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_433::NodeErrors previousErrors = node_433.errors;

            node_433.errors.output_DONE = false;

            node_433.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_433_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_433.errors.flags) {
                detail::printErrorToDebugSerial(433, node_433.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_433.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_433.errors.output_DONE) {
                    g_transaction.node_443_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_443_isNodeDirty |= g_transaction.node_433_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_433.errors.flags) {
            if (node_433.errors.output_DONE) {
                g_transaction.node_443_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #434

        if (g_transaction.node_434_hasUpstreamError) {
            g_transaction.node_444_hasUpstreamError = true;
        } else if (g_transaction.node_434_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(434);

            Node_434::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_51_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_424_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_434::NodeErrors previousErrors = node_434.errors;

            node_434.errors.output_DONE = false;

            node_434.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_434_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_434.errors.flags) {
                detail::printErrorToDebugSerial(434, node_434.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_434.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_434.errors.output_DONE) {
                    g_transaction.node_444_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_444_isNodeDirty |= g_transaction.node_434_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_434.errors.flags) {
            if (node_434.errors.output_DONE) {
                g_transaction.node_444_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #435

        if (g_transaction.node_435_hasUpstreamError) {
            g_transaction.node_445_hasUpstreamError = true;
        } else if (g_transaction.node_435_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(435);

            Node_435::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_62_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_425_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_435::NodeErrors previousErrors = node_435.errors;

            node_435.errors.output_DONE = false;

            node_435.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_435_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_435.errors.flags) {
                detail::printErrorToDebugSerial(435, node_435.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_435.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_435.errors.output_DONE) {
                    g_transaction.node_445_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_445_isNodeDirty |= g_transaction.node_435_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_435.errors.flags) {
            if (node_435.errors.output_DONE) {
                g_transaction.node_445_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #436

        if (g_transaction.node_436_hasUpstreamError) {
            g_transaction.node_446_hasUpstreamError = true;
        } else if (g_transaction.node_436_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(436);

            Node_436::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_73_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_426_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_436::NodeErrors previousErrors = node_436.errors;

            node_436.errors.output_DONE = false;

            node_436.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_436_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_436.errors.flags) {
                detail::printErrorToDebugSerial(436, node_436.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_436.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_436.errors.output_DONE) {
                    g_transaction.node_446_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_446_isNodeDirty |= g_transaction.node_436_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_436.errors.flags) {
            if (node_436.errors.output_DONE) {
                g_transaction.node_446_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #437

        if (g_transaction.node_437_hasUpstreamError) {
            g_transaction.node_447_hasUpstreamError = true;
        } else if (g_transaction.node_437_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(437);

            Node_437::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_84_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_427_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_437::NodeErrors previousErrors = node_437.errors;

            node_437.errors.output_DONE = false;

            node_437.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_437_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_437.errors.flags) {
                detail::printErrorToDebugSerial(437, node_437.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_437.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_437.errors.output_DONE) {
                    g_transaction.node_447_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_447_isNodeDirty |= g_transaction.node_437_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_437.errors.flags) {
            if (node_437.errors.output_DONE) {
                g_transaction.node_447_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #438

        if (g_transaction.node_438_hasUpstreamError) {
            g_transaction.node_448_hasUpstreamError = true;
        } else if (g_transaction.node_438_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(438);

            Node_438::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_127_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_428_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_438::NodeErrors previousErrors = node_438.errors;

            node_438.errors.output_DONE = false;

            node_438.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_438_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_438.errors.flags) {
                detail::printErrorToDebugSerial(438, node_438.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_438.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_438.errors.output_DONE) {
                    g_transaction.node_448_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_448_isNodeDirty |= g_transaction.node_438_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_438.errors.flags) {
            if (node_438.errors.output_DONE) {
                g_transaction.node_448_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #439

        if (g_transaction.node_439_hasUpstreamError) {
            g_transaction.node_449_hasUpstreamError = true;
        } else if (g_transaction.node_439_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(439);

            Node_439::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_145_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_429_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_439::NodeErrors previousErrors = node_439.errors;

            node_439.errors.output_DONE = false;

            node_439.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_439_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_439.errors.flags) {
                detail::printErrorToDebugSerial(439, node_439.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_439.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_439.errors.output_DONE) {
                    g_transaction.node_449_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_449_isNodeDirty |= g_transaction.node_439_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_439.errors.flags) {
            if (node_439.errors.output_DONE) {
                g_transaction.node_449_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #440

        if (g_transaction.node_440_hasUpstreamError) {
            g_transaction.node_450_hasUpstreamError = true;
        } else if (g_transaction.node_440_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(440);

            Node_440::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_163_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_430_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_440::NodeErrors previousErrors = node_440.errors;

            node_440.errors.output_DONE = false;

            node_440.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_440_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_440.errors.flags) {
                detail::printErrorToDebugSerial(440, node_440.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_440.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_440.errors.output_DONE) {
                    g_transaction.node_450_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_450_isNodeDirty |= g_transaction.node_440_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_440.errors.flags) {
            if (node_440.errors.output_DONE) {
                g_transaction.node_450_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #441

        if (g_transaction.node_441_hasUpstreamError) {
            g_transaction.node_451_hasUpstreamError = true;
        } else if (g_transaction.node_441_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(441);

            Node_441::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_181_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_431_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_441::NodeErrors previousErrors = node_441.errors;

            node_441.errors.output_DONE = false;

            node_441.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_441_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_441.errors.flags) {
                detail::printErrorToDebugSerial(441, node_441.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_441.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_441.errors.output_DONE) {
                    g_transaction.node_451_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_451_isNodeDirty |= g_transaction.node_441_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_441.errors.flags) {
            if (node_441.errors.output_DONE) {
                g_transaction.node_451_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #442

        if (g_transaction.node_442_hasUpstreamError) {
            g_transaction.node_452_hasUpstreamError = true;
        } else if (g_transaction.node_442_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(442);

            Node_442::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_31_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_432_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_442::NodeErrors previousErrors = node_442.errors;

            node_442.errors.output_DONE = false;

            node_442.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_442_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_442.errors.flags) {
                detail::printErrorToDebugSerial(442, node_442.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_442.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_442.errors.output_DONE) {
                    g_transaction.node_452_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_452_isNodeDirty |= g_transaction.node_442_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_442.errors.flags) {
            if (node_442.errors.output_DONE) {
                g_transaction.node_452_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #443

        if (g_transaction.node_443_hasUpstreamError) {
            g_transaction.node_453_hasUpstreamError = true;
        } else if (g_transaction.node_443_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(443);

            Node_443::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_42_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_433_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_443::NodeErrors previousErrors = node_443.errors;

            node_443.errors.output_DONE = false;

            node_443.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_443_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_443.errors.flags) {
                detail::printErrorToDebugSerial(443, node_443.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_443.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_443.errors.output_DONE) {
                    g_transaction.node_453_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_453_isNodeDirty |= g_transaction.node_443_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_443.errors.flags) {
            if (node_443.errors.output_DONE) {
                g_transaction.node_453_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #444

        if (g_transaction.node_444_hasUpstreamError) {
            g_transaction.node_454_hasUpstreamError = true;
        } else if (g_transaction.node_444_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(444);

            Node_444::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_53_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_434_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_444::NodeErrors previousErrors = node_444.errors;

            node_444.errors.output_DONE = false;

            node_444.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_444_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_444.errors.flags) {
                detail::printErrorToDebugSerial(444, node_444.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_444.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_444.errors.output_DONE) {
                    g_transaction.node_454_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_454_isNodeDirty |= g_transaction.node_444_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_444.errors.flags) {
            if (node_444.errors.output_DONE) {
                g_transaction.node_454_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #445

        if (g_transaction.node_445_hasUpstreamError) {
            g_transaction.node_455_hasUpstreamError = true;
        } else if (g_transaction.node_445_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(445);

            Node_445::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_64_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_435_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_445::NodeErrors previousErrors = node_445.errors;

            node_445.errors.output_DONE = false;

            node_445.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_445_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_445.errors.flags) {
                detail::printErrorToDebugSerial(445, node_445.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_445.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_445.errors.output_DONE) {
                    g_transaction.node_455_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_455_isNodeDirty |= g_transaction.node_445_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_445.errors.flags) {
            if (node_445.errors.output_DONE) {
                g_transaction.node_455_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #446

        if (g_transaction.node_446_hasUpstreamError) {
            g_transaction.node_456_hasUpstreamError = true;
        } else if (g_transaction.node_446_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(446);

            Node_446::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_75_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_436_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_446::NodeErrors previousErrors = node_446.errors;

            node_446.errors.output_DONE = false;

            node_446.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_446_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_446.errors.flags) {
                detail::printErrorToDebugSerial(446, node_446.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_446.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_446.errors.output_DONE) {
                    g_transaction.node_456_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_456_isNodeDirty |= g_transaction.node_446_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_446.errors.flags) {
            if (node_446.errors.output_DONE) {
                g_transaction.node_456_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #447

        if (g_transaction.node_447_hasUpstreamError) {
            g_transaction.node_457_hasUpstreamError = true;
        } else if (g_transaction.node_447_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(447);

            Node_447::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_86_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_437_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_447::NodeErrors previousErrors = node_447.errors;

            node_447.errors.output_DONE = false;

            node_447.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_447_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_447.errors.flags) {
                detail::printErrorToDebugSerial(447, node_447.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_447.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_447.errors.output_DONE) {
                    g_transaction.node_457_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_457_isNodeDirty |= g_transaction.node_447_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_447.errors.flags) {
            if (node_447.errors.output_DONE) {
                g_transaction.node_457_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #448

        if (g_transaction.node_448_hasUpstreamError) {
            g_transaction.node_458_hasUpstreamError = true;
        } else if (g_transaction.node_448_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(448);

            Node_448::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_129_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_438_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_448::NodeErrors previousErrors = node_448.errors;

            node_448.errors.output_DONE = false;

            node_448.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_448_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_448.errors.flags) {
                detail::printErrorToDebugSerial(448, node_448.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_448.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_448.errors.output_DONE) {
                    g_transaction.node_458_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_458_isNodeDirty |= g_transaction.node_448_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_448.errors.flags) {
            if (node_448.errors.output_DONE) {
                g_transaction.node_458_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #449

        if (g_transaction.node_449_hasUpstreamError) {
            g_transaction.node_459_hasUpstreamError = true;
        } else if (g_transaction.node_449_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(449);

            Node_449::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_147_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_439_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_449::NodeErrors previousErrors = node_449.errors;

            node_449.errors.output_DONE = false;

            node_449.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_449_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_449.errors.flags) {
                detail::printErrorToDebugSerial(449, node_449.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_449.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_449.errors.output_DONE) {
                    g_transaction.node_459_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_459_isNodeDirty |= g_transaction.node_449_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_449.errors.flags) {
            if (node_449.errors.output_DONE) {
                g_transaction.node_459_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #450

        if (g_transaction.node_450_hasUpstreamError) {
            g_transaction.node_460_hasUpstreamError = true;
        } else if (g_transaction.node_450_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(450);

            Node_450::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_165_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_440_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_450::NodeErrors previousErrors = node_450.errors;

            node_450.errors.output_DONE = false;

            node_450.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_450_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_450.errors.flags) {
                detail::printErrorToDebugSerial(450, node_450.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_450.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_450.errors.output_DONE) {
                    g_transaction.node_460_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_460_isNodeDirty |= g_transaction.node_450_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_450.errors.flags) {
            if (node_450.errors.output_DONE) {
                g_transaction.node_460_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #451

        if (g_transaction.node_451_hasUpstreamError) {
            g_transaction.node_461_hasUpstreamError = true;
        } else if (g_transaction.node_451_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(451);

            Node_451::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_183_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_441_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_451::NodeErrors previousErrors = node_451.errors;

            node_451.errors.output_DONE = false;

            node_451.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_451_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_451.errors.flags) {
                detail::printErrorToDebugSerial(451, node_451.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_451.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_451.errors.output_DONE) {
                    g_transaction.node_461_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_461_isNodeDirty |= g_transaction.node_451_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_451.errors.flags) {
            if (node_451.errors.output_DONE) {
                g_transaction.node_461_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #452

        if (g_transaction.node_452_hasUpstreamError) {
            g_transaction.node_462_hasUpstreamError = true;
        } else if (g_transaction.node_452_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(452);

            Node_452::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_32_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_442_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_452::NodeErrors previousErrors = node_452.errors;

            node_452.errors.output_DONE = false;

            node_452.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_452_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_452.errors.flags) {
                detail::printErrorToDebugSerial(452, node_452.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_452.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_452.errors.output_DONE) {
                    g_transaction.node_462_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_462_isNodeDirty |= g_transaction.node_452_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_452.errors.flags) {
            if (node_452.errors.output_DONE) {
                g_transaction.node_462_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #453

        if (g_transaction.node_453_hasUpstreamError) {
            g_transaction.node_463_hasUpstreamError = true;
        } else if (g_transaction.node_453_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(453);

            Node_453::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_43_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_443_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_453::NodeErrors previousErrors = node_453.errors;

            node_453.errors.output_DONE = false;

            node_453.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_453_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_453.errors.flags) {
                detail::printErrorToDebugSerial(453, node_453.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_453.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_453.errors.output_DONE) {
                    g_transaction.node_463_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_463_isNodeDirty |= g_transaction.node_453_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_453.errors.flags) {
            if (node_453.errors.output_DONE) {
                g_transaction.node_463_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #454

        if (g_transaction.node_454_hasUpstreamError) {
            g_transaction.node_464_hasUpstreamError = true;
        } else if (g_transaction.node_454_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(454);

            Node_454::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_54_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_444_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_454::NodeErrors previousErrors = node_454.errors;

            node_454.errors.output_DONE = false;

            node_454.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_454_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_454.errors.flags) {
                detail::printErrorToDebugSerial(454, node_454.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_454.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_454.errors.output_DONE) {
                    g_transaction.node_464_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_464_isNodeDirty |= g_transaction.node_454_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_454.errors.flags) {
            if (node_454.errors.output_DONE) {
                g_transaction.node_464_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #455

        if (g_transaction.node_455_hasUpstreamError) {
            g_transaction.node_465_hasUpstreamError = true;
        } else if (g_transaction.node_455_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(455);

            Node_455::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_65_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_445_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_455::NodeErrors previousErrors = node_455.errors;

            node_455.errors.output_DONE = false;

            node_455.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_455_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_455.errors.flags) {
                detail::printErrorToDebugSerial(455, node_455.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_455.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_455.errors.output_DONE) {
                    g_transaction.node_465_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_465_isNodeDirty |= g_transaction.node_455_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_455.errors.flags) {
            if (node_455.errors.output_DONE) {
                g_transaction.node_465_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #456

        if (g_transaction.node_456_hasUpstreamError) {
            g_transaction.node_466_hasUpstreamError = true;
        } else if (g_transaction.node_456_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(456);

            Node_456::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_76_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_446_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_456::NodeErrors previousErrors = node_456.errors;

            node_456.errors.output_DONE = false;

            node_456.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_456_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_456.errors.flags) {
                detail::printErrorToDebugSerial(456, node_456.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_456.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_456.errors.output_DONE) {
                    g_transaction.node_466_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_466_isNodeDirty |= g_transaction.node_456_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_456.errors.flags) {
            if (node_456.errors.output_DONE) {
                g_transaction.node_466_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #457

        if (g_transaction.node_457_hasUpstreamError) {
            g_transaction.node_467_hasUpstreamError = true;
        } else if (g_transaction.node_457_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(457);

            Node_457::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_87_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_447_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_457::NodeErrors previousErrors = node_457.errors;

            node_457.errors.output_DONE = false;

            node_457.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_457_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_457.errors.flags) {
                detail::printErrorToDebugSerial(457, node_457.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_457.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_457.errors.output_DONE) {
                    g_transaction.node_467_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_467_isNodeDirty |= g_transaction.node_457_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_457.errors.flags) {
            if (node_457.errors.output_DONE) {
                g_transaction.node_467_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #458

        if (g_transaction.node_458_hasUpstreamError) {
            g_transaction.node_468_hasUpstreamError = true;
        } else if (g_transaction.node_458_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(458);

            Node_458::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_130_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_448_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_458::NodeErrors previousErrors = node_458.errors;

            node_458.errors.output_DONE = false;

            node_458.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_458_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_458.errors.flags) {
                detail::printErrorToDebugSerial(458, node_458.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_458.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_458.errors.output_DONE) {
                    g_transaction.node_468_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_468_isNodeDirty |= g_transaction.node_458_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_458.errors.flags) {
            if (node_458.errors.output_DONE) {
                g_transaction.node_468_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #459

        if (g_transaction.node_459_hasUpstreamError) {
            g_transaction.node_469_hasUpstreamError = true;
        } else if (g_transaction.node_459_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(459);

            Node_459::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_148_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_449_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_459::NodeErrors previousErrors = node_459.errors;

            node_459.errors.output_DONE = false;

            node_459.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_459_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_459.errors.flags) {
                detail::printErrorToDebugSerial(459, node_459.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_459.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_459.errors.output_DONE) {
                    g_transaction.node_469_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_469_isNodeDirty |= g_transaction.node_459_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_459.errors.flags) {
            if (node_459.errors.output_DONE) {
                g_transaction.node_469_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #460

        if (g_transaction.node_460_hasUpstreamError) {
            g_transaction.node_470_hasUpstreamError = true;
        } else if (g_transaction.node_460_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(460);

            Node_460::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_166_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_450_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_460::NodeErrors previousErrors = node_460.errors;

            node_460.errors.output_DONE = false;

            node_460.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_460_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_460.errors.flags) {
                detail::printErrorToDebugSerial(460, node_460.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_460.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_460.errors.output_DONE) {
                    g_transaction.node_470_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_470_isNodeDirty |= g_transaction.node_460_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_460.errors.flags) {
            if (node_460.errors.output_DONE) {
                g_transaction.node_470_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #461

        if (g_transaction.node_461_hasUpstreamError) {
            g_transaction.node_471_hasUpstreamError = true;
        } else if (g_transaction.node_461_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(461);

            Node_461::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_184_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_451_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_461::NodeErrors previousErrors = node_461.errors;

            node_461.errors.output_DONE = false;

            node_461.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_461_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_461.errors.flags) {
                detail::printErrorToDebugSerial(461, node_461.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_461.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_461.errors.output_DONE) {
                    g_transaction.node_471_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_471_isNodeDirty |= g_transaction.node_461_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_461.errors.flags) {
            if (node_461.errors.output_DONE) {
                g_transaction.node_471_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #462

        if (g_transaction.node_462_hasUpstreamError) {
            g_transaction.node_472_hasUpstreamError = true;
        } else if (g_transaction.node_462_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(462);

            Node_462::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;
            ctxObj._input_BYTE = node_27_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_452_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_462::NodeErrors previousErrors = node_462.errors;

            node_462.errors.output_DONE = false;

            node_462.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_462_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_462.errors.flags) {
                detail::printErrorToDebugSerial(462, node_462.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_462.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_482_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_462.errors.output_DONE) {
                    g_transaction.node_472_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_472_isNodeDirty |= g_transaction.node_462_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_462.errors.flags) {
            if (node_462.errors.output_DONE) {
                g_transaction.node_472_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #463

        if (g_transaction.node_463_hasUpstreamError) {
            g_transaction.node_473_hasUpstreamError = true;
        } else if (g_transaction.node_463_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(463);

            Node_463::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;
            ctxObj._input_BYTE = node_38_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_453_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_463::NodeErrors previousErrors = node_463.errors;

            node_463.errors.output_DONE = false;

            node_463.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_463_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_463.errors.flags) {
                detail::printErrorToDebugSerial(463, node_463.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_463.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_483_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_463.errors.output_DONE) {
                    g_transaction.node_473_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_473_isNodeDirty |= g_transaction.node_463_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_463.errors.flags) {
            if (node_463.errors.output_DONE) {
                g_transaction.node_473_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #464

        if (g_transaction.node_464_hasUpstreamError) {
            g_transaction.node_474_hasUpstreamError = true;
        } else if (g_transaction.node_464_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(464);

            Node_464::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;
            ctxObj._input_BYTE = node_49_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_454_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_464::NodeErrors previousErrors = node_464.errors;

            node_464.errors.output_DONE = false;

            node_464.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_464_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_464.errors.flags) {
                detail::printErrorToDebugSerial(464, node_464.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_464.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_484_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_464.errors.output_DONE) {
                    g_transaction.node_474_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_474_isNodeDirty |= g_transaction.node_464_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_464.errors.flags) {
            if (node_464.errors.output_DONE) {
                g_transaction.node_474_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #465

        if (g_transaction.node_465_hasUpstreamError) {
            g_transaction.node_475_hasUpstreamError = true;
        } else if (g_transaction.node_465_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(465);

            Node_465::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;
            ctxObj._input_BYTE = node_60_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_455_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_465::NodeErrors previousErrors = node_465.errors;

            node_465.errors.output_DONE = false;

            node_465.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_465_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_465.errors.flags) {
                detail::printErrorToDebugSerial(465, node_465.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_465.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_485_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_465.errors.output_DONE) {
                    g_transaction.node_475_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_475_isNodeDirty |= g_transaction.node_465_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_465.errors.flags) {
            if (node_465.errors.output_DONE) {
                g_transaction.node_475_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #466

        if (g_transaction.node_466_hasUpstreamError) {
            g_transaction.node_476_hasUpstreamError = true;
        } else if (g_transaction.node_466_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(466);

            Node_466::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;
            ctxObj._input_BYTE = node_71_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_456_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_466::NodeErrors previousErrors = node_466.errors;

            node_466.errors.output_DONE = false;

            node_466.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_466_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_466.errors.flags) {
                detail::printErrorToDebugSerial(466, node_466.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_466.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_486_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_466.errors.output_DONE) {
                    g_transaction.node_476_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_476_isNodeDirty |= g_transaction.node_466_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_466.errors.flags) {
            if (node_466.errors.output_DONE) {
                g_transaction.node_476_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #467

        if (g_transaction.node_467_hasUpstreamError) {
            g_transaction.node_477_hasUpstreamError = true;
        } else if (g_transaction.node_467_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(467);

            Node_467::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;
            ctxObj._input_BYTE = node_82_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_457_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_467::NodeErrors previousErrors = node_467.errors;

            node_467.errors.output_DONE = false;

            node_467.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_467_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_467.errors.flags) {
                detail::printErrorToDebugSerial(467, node_467.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_467.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_487_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_467.errors.output_DONE) {
                    g_transaction.node_477_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_477_isNodeDirty |= g_transaction.node_467_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_467.errors.flags) {
            if (node_467.errors.output_DONE) {
                g_transaction.node_477_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #468

        if (g_transaction.node_468_hasUpstreamError) {
            g_transaction.node_478_hasUpstreamError = true;
        } else if (g_transaction.node_468_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(468);

            Node_468::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;
            ctxObj._input_BYTE = node_125_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_458_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_468::NodeErrors previousErrors = node_468.errors;

            node_468.errors.output_DONE = false;

            node_468.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_468_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_468.errors.flags) {
                detail::printErrorToDebugSerial(468, node_468.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_468.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_488_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_468.errors.output_DONE) {
                    g_transaction.node_478_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_478_isNodeDirty |= g_transaction.node_468_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_468.errors.flags) {
            if (node_468.errors.output_DONE) {
                g_transaction.node_478_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #469

        if (g_transaction.node_469_hasUpstreamError) {
            g_transaction.node_479_hasUpstreamError = true;
        } else if (g_transaction.node_469_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(469);

            Node_469::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;
            ctxObj._input_BYTE = node_143_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_459_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_469::NodeErrors previousErrors = node_469.errors;

            node_469.errors.output_DONE = false;

            node_469.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_469_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_469.errors.flags) {
                detail::printErrorToDebugSerial(469, node_469.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_469.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_489_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_469.errors.output_DONE) {
                    g_transaction.node_479_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_479_isNodeDirty |= g_transaction.node_469_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_469.errors.flags) {
            if (node_469.errors.output_DONE) {
                g_transaction.node_479_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #470

        if (g_transaction.node_470_hasUpstreamError) {
            g_transaction.node_480_hasUpstreamError = true;
        } else if (g_transaction.node_470_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(470);

            Node_470::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;
            ctxObj._input_BYTE = node_161_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_460_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_470::NodeErrors previousErrors = node_470.errors;

            node_470.errors.output_DONE = false;

            node_470.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_470_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_470.errors.flags) {
                detail::printErrorToDebugSerial(470, node_470.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_470.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_490_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_470.errors.output_DONE) {
                    g_transaction.node_480_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_480_isNodeDirty |= g_transaction.node_470_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_470.errors.flags) {
            if (node_470.errors.output_DONE) {
                g_transaction.node_480_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__write_byte #471

        if (g_transaction.node_471_hasUpstreamError) {
            g_transaction.node_481_hasUpstreamError = true;
        } else if (g_transaction.node_471_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(471);

            Node_471::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;
            ctxObj._input_BYTE = node_179_output_VAL;

            ctxObj._isInputDirty_SEND = g_transaction.node_461_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            Node_471::NodeErrors previousErrors = node_471.errors;

            node_471.errors.output_DONE = false;

            node_471.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_471_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            if (previousErrors.flags != node_471.errors.flags) {
                detail::printErrorToDebugSerial(471, node_471.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_471.errors.output_DONE != previousErrors.output_DONE) {
                    g_transaction.node_491_isNodeDirty = true;
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_DONE && !node_471.errors.output_DONE) {
                    g_transaction.node_481_isNodeDirty = true;
                }
            }

            // mark downstream nodes dirty
            g_transaction.node_481_isNodeDirty |= g_transaction.node_471_isOutputDirty_DONE;
        }

        // propagate errors hold by the node outputs
        if (node_471.errors.flags) {
            if (node_471.errors.output_DONE) {
                g_transaction.node_481_hasUpstreamError = true;
            }
        }
    }
    { // xod__uart__end #472

        if (g_transaction.node_472_hasUpstreamError) {
            g_transaction.node_482_hasUpstreamError = true;
        } else if (g_transaction.node_472_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(472);

            Node_472::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_342._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_462_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_472.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_472_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_482_isNodeDirty |= g_transaction.node_472_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #473

        if (g_transaction.node_473_hasUpstreamError) {
            g_transaction.node_483_hasUpstreamError = true;
        } else if (g_transaction.node_473_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(473);

            Node_473::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_345._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_463_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_473.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_473_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_483_isNodeDirty |= g_transaction.node_473_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #474

        if (g_transaction.node_474_hasUpstreamError) {
            g_transaction.node_484_hasUpstreamError = true;
        } else if (g_transaction.node_474_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(474);

            Node_474::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_348._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_464_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_474.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_474_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_484_isNodeDirty |= g_transaction.node_474_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #475

        if (g_transaction.node_475_hasUpstreamError) {
            g_transaction.node_485_hasUpstreamError = true;
        } else if (g_transaction.node_475_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(475);

            Node_475::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_351._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_465_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_475.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_475_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_485_isNodeDirty |= g_transaction.node_475_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #476

        if (g_transaction.node_476_hasUpstreamError) {
            g_transaction.node_486_hasUpstreamError = true;
        } else if (g_transaction.node_476_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(476);

            Node_476::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_354._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_466_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_476.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_476_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_486_isNodeDirty |= g_transaction.node_476_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #477

        if (g_transaction.node_477_hasUpstreamError) {
            g_transaction.node_487_hasUpstreamError = true;
        } else if (g_transaction.node_477_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(477);

            Node_477::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_357._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_467_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_477.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_477_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_487_isNodeDirty |= g_transaction.node_477_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #478

        if (g_transaction.node_478_hasUpstreamError) {
            g_transaction.node_488_hasUpstreamError = true;
        } else if (g_transaction.node_478_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(478);

            Node_478::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_360._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_468_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_478.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_478_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_488_isNodeDirty |= g_transaction.node_478_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #479

        if (g_transaction.node_479_hasUpstreamError) {
            g_transaction.node_489_hasUpstreamError = true;
        } else if (g_transaction.node_479_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(479);

            Node_479::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_363._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_469_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_479.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_479_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_489_isNodeDirty |= g_transaction.node_479_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #480

        if (g_transaction.node_480_hasUpstreamError) {
            g_transaction.node_490_hasUpstreamError = true;
        } else if (g_transaction.node_480_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(480);

            Node_480::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_366._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_470_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_480.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_480_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_490_isNodeDirty |= g_transaction.node_480_isOutputDirty_DONE;
        }

    }
    { // xod__uart__end #481

        if (g_transaction.node_481_hasUpstreamError) {
            g_transaction.node_491_hasUpstreamError = true;
        } else if (g_transaction.node_481_isNodeDirty) {
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(481);

            Node_481::ContextObject ctxObj;

            // copy data from upstream nodes into context
            ctxObj._input_UART = node_369._output_UART;

            ctxObj._isInputDirty_UPD = g_transaction.node_471_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_DONE = false;

            node_481.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction
            g_transaction.node_481_isOutputDirty_DONE = ctxObj._isOutputDirty_DONE;

            // mark downstream nodes dirty
            g_transaction.node_491_isNodeDirty |= g_transaction.node_481_isOutputDirty_DONE;
        }

    }
    { // xod__core__defer__pulse #482

        if (g_transaction.node_482_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_462.errors.output_DONE;
            error_input_IN |= node_452.errors.output_DONE;
            error_input_IN |= node_442.errors.output_DONE;
            error_input_IN |= node_432.errors.output_DONE;
            error_input_IN |= node_422.errors.output_DONE;
            error_input_IN |= node_412.errors.output_DONE;
            error_input_IN |= node_392.errors.output_DONE;
            error_input_IN |= node_372.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(482);

            Node_482::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_472_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_482::NodeErrors previousErrors = node_482.errors;

            node_482.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_482.errors.flags) {
                detail::printErrorToDebugSerial(482, node_482.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_482.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_482.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_482.errors.flags) {
            if (node_482.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #483

        if (g_transaction.node_483_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_463.errors.output_DONE;
            error_input_IN |= node_453.errors.output_DONE;
            error_input_IN |= node_443.errors.output_DONE;
            error_input_IN |= node_433.errors.output_DONE;
            error_input_IN |= node_423.errors.output_DONE;
            error_input_IN |= node_413.errors.output_DONE;
            error_input_IN |= node_394.errors.output_DONE;
            error_input_IN |= node_374.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(483);

            Node_483::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_473_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_483::NodeErrors previousErrors = node_483.errors;

            node_483.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_483.errors.flags) {
                detail::printErrorToDebugSerial(483, node_483.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_483.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_483.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_483.errors.flags) {
            if (node_483.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #484

        if (g_transaction.node_484_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_464.errors.output_DONE;
            error_input_IN |= node_454.errors.output_DONE;
            error_input_IN |= node_444.errors.output_DONE;
            error_input_IN |= node_434.errors.output_DONE;
            error_input_IN |= node_424.errors.output_DONE;
            error_input_IN |= node_414.errors.output_DONE;
            error_input_IN |= node_396.errors.output_DONE;
            error_input_IN |= node_376.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(484);

            Node_484::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_474_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_484::NodeErrors previousErrors = node_484.errors;

            node_484.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_484.errors.flags) {
                detail::printErrorToDebugSerial(484, node_484.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_484.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_484.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_484.errors.flags) {
            if (node_484.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #485

        if (g_transaction.node_485_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_465.errors.output_DONE;
            error_input_IN |= node_455.errors.output_DONE;
            error_input_IN |= node_445.errors.output_DONE;
            error_input_IN |= node_435.errors.output_DONE;
            error_input_IN |= node_425.errors.output_DONE;
            error_input_IN |= node_415.errors.output_DONE;
            error_input_IN |= node_398.errors.output_DONE;
            error_input_IN |= node_378.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(485);

            Node_485::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_475_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_485::NodeErrors previousErrors = node_485.errors;

            node_485.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_485.errors.flags) {
                detail::printErrorToDebugSerial(485, node_485.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_485.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_485.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_485.errors.flags) {
            if (node_485.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #486

        if (g_transaction.node_486_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_466.errors.output_DONE;
            error_input_IN |= node_456.errors.output_DONE;
            error_input_IN |= node_446.errors.output_DONE;
            error_input_IN |= node_436.errors.output_DONE;
            error_input_IN |= node_426.errors.output_DONE;
            error_input_IN |= node_416.errors.output_DONE;
            error_input_IN |= node_400.errors.output_DONE;
            error_input_IN |= node_380.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(486);

            Node_486::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_476_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_486::NodeErrors previousErrors = node_486.errors;

            node_486.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_486.errors.flags) {
                detail::printErrorToDebugSerial(486, node_486.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_486.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_486.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_486.errors.flags) {
            if (node_486.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #487

        if (g_transaction.node_487_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_467.errors.output_DONE;
            error_input_IN |= node_457.errors.output_DONE;
            error_input_IN |= node_447.errors.output_DONE;
            error_input_IN |= node_437.errors.output_DONE;
            error_input_IN |= node_427.errors.output_DONE;
            error_input_IN |= node_417.errors.output_DONE;
            error_input_IN |= node_402.errors.output_DONE;
            error_input_IN |= node_382.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(487);

            Node_487::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_477_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_487::NodeErrors previousErrors = node_487.errors;

            node_487.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_487.errors.flags) {
                detail::printErrorToDebugSerial(487, node_487.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_487.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_487.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_487.errors.flags) {
            if (node_487.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #488

        if (g_transaction.node_488_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_468.errors.output_DONE;
            error_input_IN |= node_458.errors.output_DONE;
            error_input_IN |= node_448.errors.output_DONE;
            error_input_IN |= node_438.errors.output_DONE;
            error_input_IN |= node_428.errors.output_DONE;
            error_input_IN |= node_418.errors.output_DONE;
            error_input_IN |= node_404.errors.output_DONE;
            error_input_IN |= node_384.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(488);

            Node_488::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_478_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_488::NodeErrors previousErrors = node_488.errors;

            node_488.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_488.errors.flags) {
                detail::printErrorToDebugSerial(488, node_488.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_488.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_488.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_488.errors.flags) {
            if (node_488.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #489

        if (g_transaction.node_489_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_469.errors.output_DONE;
            error_input_IN |= node_459.errors.output_DONE;
            error_input_IN |= node_449.errors.output_DONE;
            error_input_IN |= node_439.errors.output_DONE;
            error_input_IN |= node_429.errors.output_DONE;
            error_input_IN |= node_419.errors.output_DONE;
            error_input_IN |= node_406.errors.output_DONE;
            error_input_IN |= node_386.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(489);

            Node_489::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_479_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_489::NodeErrors previousErrors = node_489.errors;

            node_489.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_489.errors.flags) {
                detail::printErrorToDebugSerial(489, node_489.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_489.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_489.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_489.errors.flags) {
            if (node_489.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #490

        if (g_transaction.node_490_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_470.errors.output_DONE;
            error_input_IN |= node_460.errors.output_DONE;
            error_input_IN |= node_450.errors.output_DONE;
            error_input_IN |= node_440.errors.output_DONE;
            error_input_IN |= node_430.errors.output_DONE;
            error_input_IN |= node_420.errors.output_DONE;
            error_input_IN |= node_408.errors.output_DONE;
            error_input_IN |= node_388.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(490);

            Node_490::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_480_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_490::NodeErrors previousErrors = node_490.errors;

            node_490.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_490.errors.flags) {
                detail::printErrorToDebugSerial(490, node_490.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_490.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_490.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_490.errors.flags) {
            if (node_490.errors.output_OUT) {
            }
        }
    }
    { // xod__core__defer__pulse #491

        if (g_transaction.node_491_isNodeDirty) {
            bool error_input_IN = false;
            error_input_IN |= node_471.errors.output_DONE;
            error_input_IN |= node_461.errors.output_DONE;
            error_input_IN |= node_451.errors.output_DONE;
            error_input_IN |= node_441.errors.output_DONE;
            error_input_IN |= node_431.errors.output_DONE;
            error_input_IN |= node_421.errors.output_DONE;
            error_input_IN |= node_410.errors.output_DONE;
            error_input_IN |= node_390.errors.output_DONE;
            XOD_TRACE_F("Eval node #");
            XOD_TRACE_LN(491);

            Node_491::ContextObject ctxObj;

            ctxObj._error_input_IN = error_input_IN;

            // copy data from upstream nodes into context

            ctxObj._isInputDirty_IN = g_transaction.node_481_isOutputDirty_DONE;

            // initialize temporary output dirtyness state in the context,
            // where it can be modified from `raiseError` and `emitValue`
            ctxObj._isOutputDirty_OUT = false;

            Node_491::NodeErrors previousErrors = node_491.errors;

            node_491.evaluate(&ctxObj);

            // transfer possibly modified dirtiness state from context to g_transaction

            if (previousErrors.flags != node_491.errors.flags) {
                detail::printErrorToDebugSerial(491, node_491.errors.flags);

                // if an error was just raised or cleared from an output,
                // mark nearest downstream error catchers as dirty
                if (node_491.errors.output_OUT != previousErrors.output_OUT) {
                }

                // if a pulse output was cleared from error, mark downstream nodes as dirty
                // (no matter if a pulse was emitted or not)
                if (previousErrors.output_OUT && !node_491.errors.output_OUT) {
                }
            }

            // mark downstream nodes dirty
        }

        // propagate errors hold by the node outputs
        if (node_491.errors.flags) {
            if (node_491.errors.output_OUT) {
            }
        }
    }

    // Clear dirtieness and timeouts for all nodes and pins
    memset(&g_transaction, 0, sizeof(g_transaction));

    detail::clearStaleTimeout(&node_193);
    detail::clearStaleTimeout(&node_196);
    detail::clearStaleTimeout(&node_199);
    detail::clearStaleTimeout(&node_202);
    detail::clearStaleTimeout(&node_205);
    detail::clearStaleTimeout(&node_208);
    detail::clearStaleTimeout(&node_211);
    detail::clearStaleTimeout(&node_214);
    detail::clearStaleTimeout(&node_217);
    detail::clearStaleTimeout(&node_220);
    detail::clearStaleTimeout(&node_272);
    detail::clearStaleTimeout(&node_273);
    detail::clearStaleTimeout(&node_274);
    detail::clearStaleTimeout(&node_275);
    detail::clearStaleTimeout(&node_276);
    detail::clearStaleTimeout(&node_277);
    detail::clearStaleTimeout(&node_278);
    detail::clearStaleTimeout(&node_279);
    detail::clearStaleTimeout(&node_280);
    detail::clearStaleTimeout(&node_281);

    XOD_TRACE_F("Transaction completed, t=");
    XOD_TRACE_LN(millis());
}

} // namespace xod
