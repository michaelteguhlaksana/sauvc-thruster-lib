# ifndef SAUVC2020_CONFIG_H
# define SAUVC2020_CONFIG_H
/**
 * Motor Id to Arduino Pin Connection.
 */
# define THRUSTER_ID_1_PIN int(3)
# define THRUSTER_ID_2_PIN int(4)
# define THRUSTER_ID_3_PIN int(5)
# define THRUSTER_ID_4_PIN int(6)
# define THRUSTER_ID_5_PIN int(7)
# define THRUSTER_ID_6_PIN int(8)
# define THRUSTER_ID_7_PIN int(9)
# define THRUSTER_ID_8_PIN int(10)
# define THRUSTER_ID_TO_ARDUINO_PIN {1, THRUSTER_ID_1_PIN}, \
                                    {2, THRUSTER_ID_2_PIN}, \
                                    {3, THRUSTER_ID_3_PIN}, \
                                    {4, THRUSTER_ID_4_PIN}, \
                                    {5, THRUSTER_ID_5_PIN}, \
                                    {6, THRUSTER_ID_6_PIN}, \
                                    {7, THRUSTER_ID_7_PIN}, \
                                    {8, THRUSTER_ID_8_PIN}

/**
 * ESC Input Value Safety Limit
 */
# define MAX_ESC_INPUT int(1700)
# define MIN_ESC_INPUT int(1300)

/**
 * ESC Input Value for Stop Signal
 */
# define ESC_INPUT_STOP int(1500)

//AUV changes position in 2D with body direction fixed
enum GoMotion
{
  FORWARD,
  BACKWARD,
  STOP
};

enum TranslateMotion
{
  LEFT,
  RIGHT,
  STOP
};

//AUV changes position in z axis(submerge or surface) with body direction fixed
enum VerticalMotion
{
  SUBMERGE,
  SURFACE,
  STOP
};

//AUV changes body direction with position in space fixed
enum PitchMotion
{
  FORWARD,
  BACKWARD,
  STOP
};

enum RollMotion
{
  RIGHT,
  LEFT,
  STOP
};

enum RotateMotion
{
  RIGHT,
  LEFT,
  STOP
};

/**
 * ESC Input Value of motors for each motion.
 */
# define POS_INPUT_GO int(1700)
# define NEG_INPUT_GO int(1400)
# define ID_TO_ESC_GO_FORWARD {1, POS_INPUT_GO}, \
                              {2, POS_INPUT_GO}, \
                              {3, POS_INPUT_GO}, \
                              {4, POS_INPUT_GO}
# define ID_TO_ESC_GO_BACKWARD {1, NEG_INPUT_GO}, \
                               {2, NEG_INPUT_GO}, \
                               {3, NEG_INPUT_GO}, \
                               {4, NEG_INPUT_GO}
# define ID_TO_ESC_GO_STOP {1, ESC_INPUT_STOP}, \
                           {2, ESC_INPUT_STOP}, \
                           {3, ESC_INPUT_STOP}, \
                           {4, ESC_INPUT_STOP}

# define POS_INPUT_VERTICAL int(1600)
# define NEG_INPUT_VERTICAL int(1400)
# define ID_TO_ESC_VERTICAL_SUBMERGE {5, NEG_INPUT_VERTICAL}, \
                                     {6, POS_INPUT_VERTICAL}, \
                                     {7, POS_INPUT_VERTICAL}, \
                                     {8, NEG_INPUT_VERTICAL}
# define ID_TO_ESC_VERTICAL_SURFACE {5, POS_INPUT_VERTICAL}, \
                                    {6, NEG_INPUT_VERTICAL}, \
                                    {7, NEG_INPUT_VERTICAL}, \
                                    {8, POS_INPUT_VERTICAL}
# define ID_TO_ESC_VERTICAL_STOP {5, ESC_INPUT_STOP}, \
                                 {6, ESC_INPUT_STOP}, \
                                 {7, ESC_INPUT_STOP}, \
                                 {8, ESC_INPUT_STOP}

# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROTATE_LEFT {1, 1600}, \
                                                  {2, 1400}, \
                                                  {3, 1600}, \
                                                  {4, 1400}, \
                                                  {5, ESC_INPUT_STOP}, \
                                                  {6, ESC_INPUT_STOP}, \
                                                  {7, ESC_INPUT_STOP}, \
                                                  {8, ESC_INPUT_STOP}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROTATE_RIGHT {1, 1400}, \
                                                   {2, 1600}, \
                                                   {3, 1400}, \
                                                   {4, 1600}, \
                                                   {5, ESC_INPUT_STOP}, \
                                                   {6, ESC_INPUT_STOP}, \
                                                   {7, ESC_INPUT_STOP}, \
                                                   {8, ESC_INPUT_STOP}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_TRANSLATE_LEFT {1, 1600}, \
                                                     {2, 1400}, \
                                                     {3, 1400}, \
                                                     {4, 1600}, \
                                                     {5, ESC_INPUT_STOP}, \
                                                     {6, ESC_INPUT_STOP}, \
                                                     {7, ESC_INPUT_STOP}, \
                                                     {8, ESC_INPUT_STOP}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_TRANSLATE_RIGHT {1, 1400}, \
                                                      {2, 1600}, \
                                                      {3, 1600}, \
                                                      {4, 1400}, \
                                                      {5, ESC_INPUT_STOP}, \
                                                      {6, ESC_INPUT_STOP}, \
                                                      {7, ESC_INPUT_STOP}, \
                                                      {8, ESC_INPUT_STOP}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROLL_LEFT {1, ESC_INPUT_STOP}, \
                                                {2, ESC_INPUT_STOP}, \
                                                {3, ESC_INPUT_STOP}, \
                                                {4, ESC_INPUT_STOP}, \
                                                {5, 1400}, \
                                                {6, 1400}, \
                                                {7, 1600}, \
                                                {8, 1600}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_ROLL_RIGHT {1, ESC_INPUT_STOP}, \
                                                 {2, ESC_INPUT_STOP}, \
                                                 {3, ESC_INPUT_STOP}, \
                                                 {4, ESC_INPUT_STOP}, \
                                                 {5, 1600}, \
                                                 {6, 1600}, \
                                                 {7, 1400}, \
                                                 {8, 1400}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_PITCH_FORWARD {1, ESC_INPUT_STOP}, \
                                                    {2, ESC_INPUT_STOP}, \
                                                    {3, ESC_INPUT_STOP}, \
                                                    {4, ESC_INPUT_STOP}, \
                                                    {5, 1400}, \
                                                    {6, 1600}, \
                                                    {7, 1400}, \
                                                    {8, 1600}
# define THRUSTER_ID_TO_ESC_INPUT_FOR_PITCH_BACKWARD {1, ESC_INPUT_STOP}, \
                                                     {2, ESC_INPUT_STOP}, \
                                                     {3, ESC_INPUT_STOP}, \
                                                     {4, ESC_INPUT_STOP}, \
                                                     {5, 1600}, \
                                                     {6, 1400}, \
                                                     {7, 1600}, \
                                                     {8, 1400}
# endif
