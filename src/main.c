#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

//UART
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

//Ethernet
#include <zephyr/net/net_core.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_ip.h>

//Button
#include <zephyr/input/input.h>
#define BUTTON_NODE DT_ALIAS(sw0)

#include <string.h>
K_FIFO_DEFINE(fifo_stg);
K_FIFO_DEFINE(fifo_eth);

//UART DEF
#include <zephyr/sys/time_units.h>
#define UART6_NODE DT_NODELABEL(usart6)
#define UART2_NODE DT_NODELABEL(usart2)
const struct device *uart2 = DEVICE_DT_GET(UART2_NODE);
const struct device *uart6 = DEVICE_DT_GET(UART6_NODE);

//LEDS
#define LED0_NODE DT_ALIAS(led0)  // Alias pro LED1 na desce
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec leds[3] = {
GPIO_DT_SPEC_GET(LED0_NODE, gpios),  // Index 0: červená (r)
GPIO_DT_SPEC_GET(LED1_NODE, gpios),  // Index 1: zelená (g)
GPIO_DT_SPEC_GET(LED2_NODE, gpios)   // Index 2: modrá (b)
};

//ETH
#define SERVER_IP "192.168.1.136"  // IP adresa vašeho PC
#define SERVER_PORT 12345         // Port, na kterém PC naslouchá

//STORAGE
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>
#include <zephyr/device.h>

//TEMP SENSOR
#include <zephyr/drivers/sensor.h>
#define DIE_TEMP DEVICE_DT_GET(DT_NODELABEL(die_temp))
//Button
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE,gpios);
struct k_poll_signal button_signal;
struct gpio_callback button_cb_data;

struct k_poll_event events[1] = {
K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
		K_POLL_MODE_NOTIFY_ONLY,
		&button_signal), };

//ASYN CALLBACK/UART DATA RECIEVER
#define BUFFER_SIZE 32
uint8_t rx_buf[BUFFER_SIZE];
//uint8_t rx_buf2[BUFFER_SIZE];
//char sel_buf=1;
char emsg[32];
int index = 0;
static void uart_callback(const struct device *dev, struct uart_event *evt,
		void *user_data) {
	switch (evt->type) {
	case UART_RX_RDY:
		//printk("Temp recieve= %c\n", *(evt->data.rx.buf + evt->data.rx.offset));
		emsg[index] = *(evt->data.rx.buf + evt->data.rx.offset);
		index++;
		if (*(evt->data.rx.buf + evt->data.rx.offset) == '\0') {
			char *mmsg = k_malloc(index);
			memcpy(mmsg, emsg, index);
			k_fifo_put(&fifo_stg, mmsg);
			mmsg = k_malloc(index);
			memcpy(mmsg, emsg, index);
			index = 0;
			k_fifo_put(&fifo_eth, mmsg);
			printk("Entire Message recieved\n");
		}
		break;
	case UART_RX_BUF_REQUEST:
		uart_rx_buf_rsp(dev, rx_buf, BUFFER_SIZE);
		break;
	case UART_RX_BUF_RELEASED:
		memset(evt->data.rx_buf.buf, 0, BUFFER_SIZE);
		break;
	}
}

//TESTING UART SENDER
void sender_function(void *arg1, void *arg2, void *arg3) {
	int ret;
	char temp_str[20];
	struct sensor_value val;

	while (1) {
		ret = k_poll(events, 1, K_FOREVER);
		if (ret == 0) {
			// Fetch sensor samples
			ret = sensor_sample_fetch(DIE_TEMP);
			if (ret) {
				snprintf(temp_str, sizeof(temp_str), "CPU TEMP: Error");
			} else {
				// Get temperature data
				ret = sensor_channel_get(DIE_TEMP, SENSOR_CHAN_DIE_TEMP, &val);
				if (ret) {
					snprintf(temp_str, sizeof(temp_str), "CPU TEMP: Error");
				} else {
					snprintf(temp_str, sizeof(temp_str), "CPU TEMP: %.1f C",
							sensor_value_to_double(&val));
				}
			}
			for (int i = 0; temp_str[i] != '\0'; i++) {
				uart_poll_out(uart2, temp_str[i]);
				k_msleep(10);
			}
			uart_poll_out(uart2, '\0');
			k_poll_signal_reset(&button_signal);
			events[0].state = K_POLL_STATE_NOT_READY;
		}
	}
}

void button_callback(const struct device *dev, struct gpio_callback *cb,
		uint32_t pins) {
	k_poll_signal_raise(&button_signal, 1);
	printk("BUTTON DOWN\n");
}

// LED

void led_function(void *arg1, void *arg2, void *arg3) {
	for (int i = 0; i < 3; i++) {
		int ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return;
		}
		k_msleep(500);
		gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
		if (i == 2) {
			i = -1;
		}
	}

}
//STORAGE
#define MKFS_FS_TYPE FS_FATFS
#define MKFS_FLAGS 0

#define DISK_NAME_PHYS "SD"
#define MOUNT_POINT "/"DISK_NAME_PHYS":"
#define FILE_PATH "/SD:/soubor.txt"
static FATFS fat_fs;
static struct fs_mount_t sd_mount =
		{ .type = MKFS_FS_TYPE, .fs_data = &fat_fs, .mnt_point =
				(char*) MOUNT_POINT, .storage_dev = (void*) DISK_NAME_PHYS, };
void stg_function(void *arg1, void *arg2, void *arg3) {
	struct fs_file_t file;
	int ret = 0;
	char *msg;
	const char newline = '\n';
	ret = disk_access_init("SD");
	if (ret != 0) {
		printk("Chyba inicializace disku: %d\n", ret);
		return;
	}

	ret = fs_mount(&sd_mount);
	if (ret != 0) {
		printk("Chyba připojení FS: %d\n", ret);

		printk("Formátování SD karty na FS...\n");
		ret = fs_mkfs(FS_FATFS, (uintptr_t) &sd_mount, NULL, 0);
		if (ret != 0) {
			printk("Chyba formátování: %d\n", ret);
			return;
		}

		ret = fs_mount(&sd_mount);
		if (ret != 0) {
			printk("Chyba připojení po formátování: %d\n", ret);
			return;
		}
	}

	printk("FS připojen úspěšně!\n");

	while (1) {
		msg = k_fifo_get(&fifo_stg, K_FOREVER);

		fs_file_t_init(&file);

		ret = fs_open(&file, FILE_PATH, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
		if (ret != 0) {
			printk("Chyba při otevírání souboru: %d\n", ret);
			k_free(msg); /* Uvolnění paměti pro msg */
			fs_unmount(&sd_mount);
			return;
		}

		ret = fs_write(&file, msg, strlen(msg));
		if (ret < 0) {
			printk("Chyba při zápisu msg do souboru: %d\n", ret);
		} else {
			ret = fs_write(&file, &newline, sizeof(newline));
			if (ret < 0) {
				printk("Chyba při zápisu nového řádku: %d\n", ret);
			} else {
				printk("Text z msg úspěšně zapsán do %s\n", FILE_PATH);
			}
		}

		fs_close(&file);
		k_free(msg);
	}
	fs_unmount(&sd_mount);
}

//ETH
void eth_function(void *arg1, void *arg2, void *arg3) {
	int sock;
	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(SERVER_PORT);
	char *msg;
	net_addr_pton(AF_INET, SERVER_IP, &addr.sin_addr);
	while (1) {
		sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (!(sock < 0)) {
			msg = k_fifo_get(&fifo_eth, K_FOREVER);
			zsock_sendto(sock, msg, strlen(msg), 0, (struct sockaddr*) &addr,
					sizeof(addr));
			k_free(msg);
			zsock_close(sock);
		}
	}
}

//defined Threads

K_THREAD_DEFINE(sender_thread_id, 1024, sender_function, NULL, NULL, NULL, 0, 0,0);
K_THREAD_DEFINE(eth_thread_id, 2048, eth_function, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(led_thread_id, 1024, led_function, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(stg_thread_id, 2048, stg_function, NULL, NULL, NULL, 0, 0, 0);

int main(void) {
	//Button setup
	gpio_pin_configure_dt(&button, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
	gpio_init_callback(&button_cb_data, button_callback, BIT(button.pin));
	gpio_add_callback_dt(&button, &button_cb_data);
	k_poll_signal_init(&button_signal);
	//uart setup
	uart_callback_set(uart6, uart_callback, NULL);
	uart_rx_enable(uart6, rx_buf, BUFFER_SIZE, 0);
	while (1) {
		k_msleep(100);
	}
	return 0;
}
