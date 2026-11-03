#include "runtime.hpp"
#include "generated_board.hpp"
#include "S3Scanout.hpp"
#include "PixelOps.hpp"
#include "generated_scene.hpp"
#include "generated_inputs.hpp"
#include <driver/gpio.h>
#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <algorithm>
#include <cstring>

#if JET_AUDIO
#include <driver/i2s_std.h>
#include <atomic>
#include "audio_mixer.hpp"

// Core 0: mixes a block and hands it to the I2S DMA, which blocks until there is room, so it sleeps most of the time.
namespace audio {
constexpr size_t BlockFrames = 256;
jetstudio::AudioMixer mixer;
i2s_chan_handle_t channel = nullptr;
std::atomic<uint32_t> mix_max_us{0}; // the longest block since the last log line

void loop(void*) {
    static int16_t block[BlockFrames];
    for (;;) {
        const int64_t start = esp_timer_get_time();
        mixer.mix(block, BlockFrames);
        const auto took = uint32_t(esp_timer_get_time() - start);
        if (took > mix_max_us.load(std::memory_order_relaxed)) mix_max_us.store(took, std::memory_order_relaxed);
        size_t written = 0;
        if (i2s_channel_write(channel, block, sizeof block, &written, portMAX_DELAY) != ESP_OK) vTaskDelay(1);
    }
}

bool start() {
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan.dma_desc_num = 4;
    chan.dma_frame_num = BlockFrames;
    chan.auto_clear_after_cb = true; // silence if a block comes late
    if (i2s_new_channel(&chan, &channel, nullptr) != ESP_OK) return false;
    i2s_std_config_t std_config{};
    std_config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(jetstudio::AudioSampleRate);
    std_config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    std_config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH; // the NS4168 plays one channel, chosen by its CTRL pin: send both
    std_config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_config.gpio_cfg.bclk = gpio_num_t(Board::audioBclk);
    std_config.gpio_cfg.ws = gpio_num_t(Board::audioLrclk);
    std_config.gpio_cfg.dout = gpio_num_t(Board::audioDout);
    std_config.gpio_cfg.din = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(channel, &std_config) != ESP_OK || i2s_channel_enable(channel) != ESP_OK) return false;
    mixer.set_max_db(Board::audioMaxDb);
    return xTaskCreatePinnedToCore(loop, "jet_audio", 3072, nullptr, 3, nullptr, 0) == pdPASS;
}
} // namespace audio
#endif

// Same frame pipeline as CubeCoders' JetExamples, with half-width interlaced fields and no z-buffer (firmware/config).
// Core 1, game task: scripts, transforms, frame setup and the upper raster band.
// Core 0, raster worker: the lower raster band.
// Core 0, display task: pushes the previous field row by row through the DMA queue.
// It sleeps while DMA runs, so the raster worker gets core 0 most of the time.
// Two field buffers: one is drawn while the other is sent.

// Rarely used node data (extra fields, body state) goes to PSRAM: small blocks would otherwise take internal RAM
// from the buffers that drawing touches every frame.
void* jetstudio::cold_alloc(size_t bytes) {
    void* p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : malloc(bytes);
}
void jetstudio::cold_free(void* p) { heap_caps_free(p); }

namespace {
constexpr const char* Tag = "JetStudio";
// The release profile (sdkconfig.release) has no console: the log is just drained and no frame stats are computed.
#if defined(CONFIG_ESP_CONSOLE_NONE)
constexpr bool SerialLog = false;
#else
constexpr bool SerialLog = true;
#endif
static_assert(FIELD_BUFFERS && HALF_WIDTH_BUFFERS && !Z_BUFFERING, "the firmware expects firmware/config/JetConfig.hpp");

int64_t raster_time = 0; // accumulated for the statistics line

// Jet split rendering: the game task rasterizes the rows above `split`, the worker the rows below.
// The split moves work away from the slower band and stays even, like JetExamples' S3RasterWorker.
namespace raster {
TaskHandle_t worker = nullptr;
SemaphoreHandle_t done = nullptr;
Renderer::Scene* job = nullptr;
std::vector<uint8_t> upper, lower;
int height = 0, split = 0;
int64_t lower_time = 0;

void worker_loop(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t start = esp_timer_get_time();
        job->rasterizeBand(split, height, lower.data());
        lower_time = esp_timer_get_time() - start;
        xSemaphoreGive(done);
    }
}

void execute(Renderer::Scene& scene) {
    const int64_t start = esp_timer_get_time();
    if (!worker || scene.lastFrameDrawnTriangles <= 0) {
        scene.rasterizeBand(0, height);
        raster_time += esp_timer_get_time() - start;
        return;
    }
    // One flag array per band, so a triangle drawn by both bands is counted once (Jet's parallel contract).
    upper.assign(size_t(scene.lastFrameDrawnTriangles), 0);
    lower.assign(size_t(scene.lastFrameDrawnTriangles), 0);
    job = &scene;
    xTaskNotifyGive(worker);
    scene.rasterizeBand(0, split, upper.data());
    const int64_t upper_time = esp_timer_get_time() - start;
    xSemaphoreTake(done, portMAX_DELAY);
    if (lower_time > upper_time + 250) split = std::min(height - 32, split + 4);
    else if (upper_time > lower_time + 250) split = std::max(32, split - 4);
    int count = 0;
    for (size_t i = 0; i < upper.size(); ++i) count += (upper[i] | lower[i]) != 0;
    scene.lastFrameRasterizedTriangles = count;
    raster_time += esp_timer_get_time() - start;
}

bool start(int screen_height) {
    height = screen_height;
    split = (height / 2) & ~1;
    done = xSemaphoreCreateBinary();
    return done && xTaskCreatePinnedToCore(worker_loop, "jet_raster", 8192, nullptr, 1, &worker, 0) == pdPASS;
}
} // namespace raster

// Display task, one field per notification.
// It sets up the scanout itself because the DMA queue interrupt has to live on the producer's core.
namespace present {
TaskHandle_t task = nullptr;
SemaphoreHandle_t idle = nullptr; // given when the previous field has been sent
LGFX* display = nullptr;
uint16_t* rows[2] = {};
S3Scanout scanout;
bool queued = false;
const uint16_t* field = nullptr;
int parity = 0, width = 0, height = 0, field_width = 0;
std::vector<jetstudio::OverlayItem> overlay; // 2D nodes of the field being sent
int overlay_top = 0, overlay_bottom = 0;
int64_t send_time = 0;

void set_overlay(const std::vector<jetstudio::OverlayItem>& items) {
    overlay = items;
    overlay_top = height;
    overlay_bottom = 0;
    for (const auto& it : overlay) {
        overlay_top = std::min(overlay_top, std::max(0, it.y));
        overlay_bottom = std::max(overlay_bottom, std::min(height, it.y + it.h));
    }
}

// Field row (half width, native byte order) -> panel row (doubled, swapped) + HUD.
void prepare_row(uint16_t* out, int y) {
    Renderer::expandSwapRGB565(out, field + size_t(y >> 1) * field_width, size_t(field_width));
    if (y >= overlay_top && y < overlay_bottom) jetstudio::compose_overlay_row(overlay, out, y, width, true);
}

void send_field() {
    if (queued) {
        scanout.begin(parity);
        for (int y = parity; y < height; y += 2) {
            while (!scanout.free_slots()) { scanout.kick(); scanout.wait_for_space(); }
            prepare_row(scanout.next_row(), y);
            scanout.publish();
            if (!scanout.free_slots()) scanout.kick();
        }
        scanout.kick();
        scanout.finish();
        return;
    }
    // Fallback: ping-pong rows, LGFX waits for each DMA transfer.
    int current = 0;
    display->startWrite();
    for (int y = parity; y < height; y += 2) {
        prepare_row(rows[current], y);
        display->setAddrWindow(0, y, width, 1);
        display->writePixelsDMA(rows[current], width, false);
        current ^= 1;
    }
    display->endWrite();
}

void loop(void*) {
    queued = !Board::parallelBus && scanout.init(display, rows[0], rows[1], unsigned(width), unsigned(height));
    xSemaphoreGive(idle);
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t start = esp_timer_get_time();
        send_field();
        send_time += esp_timer_get_time() - start;
        xSemaphoreGive(idle);
    }
}

bool start(LGFX* target, int screen_width, int screen_height) {
    display = target;
    width = screen_width;
    height = screen_height;
    field_width = width / 2;
    for (auto& row : rows) {
        row = (uint16_t*)heap_caps_aligned_alloc(16, size_t(width) * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!row) return false;
    }
    // Prime LGFX's screen-width DMA descriptor before the queued interrupt uses it.
    std::memset(rows[0], 0, size_t(width) * sizeof(uint16_t));
    display->startWrite();
    display->setAddrWindow(0, 0, width, 1);
    display->writePixelsDMA(rows[0], width, false);
    display->waitDMA();
    display->endWrite();
    idle = xSemaphoreCreateBinary();
    if (!idle || xTaskCreatePinnedToCore(loop, "jet_present", 4096, nullptr, 2, &task, 0) != pdPASS) return false;
    xSemaphoreTake(idle, portMAX_DELAY); // scanout initialized
    xSemaphoreGive(idle);
    return true;
}
} // namespace

// Buttons wire the pin to GND and use the internal pull-up
void configure_buttons() {
    for (int pin : input_pins) {
        if (pin < 0) continue;
        gpio_config_t config{};
        config.pin_bit_mask = 1ULL << pin;
        config.mode = GPIO_MODE_INPUT;
        config.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&config);
    }
}

// Each finger presses the actions whose zone it is in. Two fingers: steer and fire at the same time.
uint32_t read_touch(LGFX& display, int width, int height) {
    uint32_t mask = 0;
#if JET_TOUCH_ZONES
    lgfx::touch_point_t points[2];
    const int count = display.getTouch(points, 2);
    for (int i = 0; i < count; ++i) {
        const int x = points[i].x * 100 / width, y = points[i].y * 100 / height;
        for (const auto& zone : touch_zones)
            if (x >= zone.left && x < zone.right && y >= zone.top && y < zone.bottom) mask |= 1u << zone.bit;
    }
#endif
    return mask;
}

uint32_t read_buttons() {
    uint32_t mask = 0;
    for (int i = 0; i < int(sizeof(input_pins) / sizeof(input_pins[0])); ++i)
        if (input_pins[i] >= 0 && gpio_get_level(gpio_num_t(input_pins[i])) == 0) mask |= 1u << i;
    return mask;
}


void print_log(jetstudio::Runtime& runtime) {
    for (const auto& line : runtime.take_log()) {
        if (!SerialLog) continue;
        if (line.error && !line.path.empty()) ESP_LOGE(Tag, "%s:%d: %s", line.path.c_str(), line.line, line.text.c_str());
        else if (line.error) ESP_LOGE(Tag, "%s", line.text.c_str());
        else ESP_LOGI(Tag, "%s", line.text.c_str());
    }
}

void stop(const char* reason) {
    ESP_LOGE(Tag, "%s", reason);
    vTaskDelete(nullptr);
}

// Own task because the script VM and frame setup need more stack than the default main task has.
void game(void*) {
    static LGFX display;
    if (!display.init()) stop("Display initialization failed");
    display.setRotation(Board::rotation);
    display.fillScreen(0);
    // Scene data (meshes, Jet objects, textures) goes to PSRAM like in the demos.
    // Internal RAM stays free for the fields, the DMA rows and Jet's transform scratch.
    heap_caps_malloc_extmem_enable(1024);
    std::string error;
    int width = 0, height = 0;
    if (!jetstudio::pack_resolution(scene_data, sizeof(scene_data), width, height)) {
        ESP_LOGE(Tag, "Invalid scene: export the project again");
        vTaskDelete(nullptr);
    }
    if (width != Board::screenWidth || height != Board::screenHeight) {
        ESP_LOGE(Tag, "The scene is %dx%d, the display (%s) %dx%d: export it with the project's hardware settings",
            width, height, Board::name, Board::screenWidth, Board::screenHeight);
        vTaskDelete(nullptr);
    }
    // Allocate the fields before the scene so they get internal DMA RAM when possible.
    // PSRAM works too, the scanout copies the rows with the CPU anyway, it's just slower.
    const size_t field_bytes = size_t(width / jetstudio::BufferColumnStep) * (height / jetstudio::BufferRowStep) * sizeof(uint16_t);
    uint16_t* fields[2];
    bool internal = true;
    for (auto& f : fields) {
        f = (uint16_t*)heap_caps_aligned_alloc(16, field_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!f) {
            internal = false;
            f = (uint16_t*)heap_caps_aligned_alloc(16, field_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (!f) stop("Not enough RAM for the field buffers");
        std::memset(f, 0, field_bytes);
    }
    if (!present::start(&display, width, height)) stop("Display task not started");
    // The package stays in flash: only the scene being played is decoded into RAM.
    static jetstudio::Runtime runtime;
    if (!runtime.load_pack(scene_data, sizeof(scene_data), error, false, true)) {
        ESP_LOGE(Tag, "Invalid scene: %s", error.c_str());
        vTaskDelete(nullptr);
    }
#if JET_AUDIO
    if (audio::start()) runtime.set_audio(&audio::mixer);
    else ESP_LOGW(Tag, "I2S not started: the game runs silent");
#endif
    std::vector<jetstudio::ScriptDiagnostic> diagnostics;
    if (!runtime.start(diagnostics)) {
        for (const auto& d : diagnostics)
            ESP_LOGE(Tag, "%s:%d:%d %s", d.path.c_str(), d.line, d.column, d.message.c_str());
        vTaskDelete(nullptr);
    }
    heap_caps_malloc_extmem_enable(CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL);
    print_log(runtime);
    configure_buttons();
    if (raster::start(runtime.height())) runtime.set_raster_executor(raster::execute);
    else ESP_LOGW(Tag, "Raster worker not started: single-core rendering");
    ESP_LOGI(Tag, "%s: %dx%d, fields of %dx%d in %s, DMA queue %s", Board::name, runtime.width(), runtime.height(),
        runtime.buffer_width(), runtime.buffer_rows(), internal ? "internal RAM" : "PSRAM (slower)", present::queued ? "on" : "off (polling)");
    ESP_LOGI(Tag, "Scene loaded: %u nodes (%u bytes), %u meshes, %u scripts; internal free %u, PSRAM free %u",
        unsigned(runtime.node_count()), unsigned(runtime.node_bytes()), unsigned(runtime.mesh_count()), unsigned(runtime.description().scripts.size()),
        unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)), unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));

    int64_t previous = esp_timer_get_time(), total = 0, scripts_time = 0, rendering = 0, waiting = 0, yielded = previous;
    unsigned count = 0;
    for (;;) {
        const int64_t start = esp_timer_get_time();
        const int parity = runtime.next_field();
        runtime.set_framebuffer(fields[parity]);
        runtime.set_input(read_buttons() | read_touch(display, width, height));
        runtime.update(float(start - previous) / 1000000.0f);
        const int64_t updated = esp_timer_get_time();
        previous = start;
        runtime.render();
        const int64_t rendered = esp_timer_get_time();
        // Wait until the previous field is out, then send this one while the next is drawn.
        xSemaphoreTake(present::idle, portMAX_DELAY);
        present::field = fields[parity];
        present::parity = parity;
        present::set_overlay(runtime.overlay());
        xTaskNotifyGive(present::task);
        print_log(runtime);
        scripts_time += updated - start;
        rendering += rendered - updated;
        waiting += esp_timer_get_time() - rendered;
        total += esp_timer_get_time() - start;
        if (SerialLog && ++count == 60) {
            // The editor's Serial monitor parses the first part, keep the format.
            ESP_LOGI(Tag, "Frame %.2f ms (scripts %.2f ms); %d raster tris; free heap %u, PSRAM %u; render %.2f ms (raster %.2f, split %d), field send %.2f ms (waited %.2f); internal %u",
                total / 60000.0, scripts_time / 60000.0, runtime.triangles(), unsigned(esp_get_free_heap_size()),
                unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)), rendering / 60000.0, raster_time / 60000.0, raster::split,
                present::send_time / 60000.0, waiting / 60000.0, unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
#if JET_AUDIO
            ESP_LOGI(Tag, "Audio mix max %u us of %u us per block", unsigned(audio::mix_max_us.exchange(0, std::memory_order_relaxed)),
                unsigned(audio::BlockFrames * 1000000 / jetstudio::AudioSampleRate));
#endif
            count = 0; total = 0; scripts_time = 0; rendering = 0; waiting = 0; raster_time = 0; present::send_time = 0;
        }
        // The task watchdog wants IDLE1 to run on this core.
        // At FREERTOS_HZ=100 a 1 ms delay is 0 ticks and never blocks, so yield a full tick every half second.
        // JetExamples' FramePacer does the same when idle starves.
        if (start - yielded > 500000) {
            vTaskDelay(1);
            yielded = esp_timer_get_time();
        }
    }
}
} // namespace

extern "C" void app_main() {
    xTaskCreatePinnedToCore(game, "jet_game", 32 * 1024, nullptr, 5, nullptr, 1);
}
