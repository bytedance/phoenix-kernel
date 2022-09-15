// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#ifdef __CAMERA_EDITOR__
#include "pico_camera_sysfs_utils.h"
#endif


#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
uint64_t  Fsin_boot_timestamp = 0;
uint32_t  Fsin_gpio = 1126;
int Fsin_gpio_irq = 0;
int Fsin_irq_register = 0;
uint8_t camera_streamoff_status = 0;
#endif
int eyetracking_led_always_on = 0;/*Modify by PICO_Driver sophia.wang for bug57941: add eyetracking led always on mode for cit test start*/
int ov680_ctrl_value = 0;/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400*/

/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--start*/
#define STROBE_REG_COUNT 2
struct cam_registers_table ov680_strobe_regs[STROBE_REG_COUNT] = {
    {0xe0,0x3009, 0x08},
    {0xe0,0x3027, 0x08},
};
#define ALTERNANT_REG_COUNT 2
struct cam_registers_table ov680_alternant_regs[ALTERNANT_REG_COUNT] = {
    {0xc2,0x3b81, 0x55},
    {0xc4,0x3b81, 0xaa},
};

static void cam_ov680_write_regs(struct cam_sensor_ctrl_t *s_ctrl, struct cam_registers_table *regs ,int size)
{
    struct cam_sensor_i2c_reg_setting i2c_settings;
    struct cam_sensor_i2c_reg_array reg_setting[1];
    struct cam_registers_table *regs_in;
    int32_t rc = 0;
    int i = 0;

	CAM_INFO(CAM_SENSOR,"sophia size = %d",size);

    for(i = 0;i < size; i++)
    {
        s_ctrl->io_master_info.cci_client->sid = 0x35;
        i2c_settings.reg_setting = reg_setting;
		regs_in = regs +i;

        i2c_settings.reg_setting[0].reg_addr = 0x6b20;
        i2c_settings.reg_setting[0].reg_data = regs_in->slave_addr;
        i2c_settings.reg_setting[0].data_mask = 0;
        i2c_settings.size = 1;
        i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
        i2c_settings.reg_setting[0].delay = 0;
        i2c_settings.delay = 1;

        rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings);
        if(rc < 0)
            CAM_ERR(CAM_SENSOR, "Write ov680 reg  failed. ID:%d, addr:0x%x",s_ctrl->io_master_info.cci_client->sid,i2c_settings.reg_setting[0].reg_addr);


        i2c_settings.reg_setting[0].reg_addr = 0x6b21;
        i2c_settings.reg_setting[0].reg_data =( regs_in->reg_addr & 0xFF00) >> 8;
        i2c_settings.reg_setting[0].data_mask = 0;
        i2c_settings.size = 1;
        i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
        i2c_settings.reg_setting[0].delay = 0;
        i2c_settings.delay = 1;

        rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings);
        if(rc < 0)
            CAM_ERR(CAM_SENSOR, "Write ov680 reg  failed. ID:%d, addr:0x%x",s_ctrl->io_master_info.cci_client->sid,i2c_settings.reg_setting[0].reg_addr);

        i2c_settings.reg_setting = reg_setting;
        i2c_settings.reg_setting[0].reg_addr = 0x6b22;
        i2c_settings.reg_setting[0].reg_data = regs_in->reg_addr& 0xFF;
        i2c_settings.reg_setting[0].data_mask = 0;
        i2c_settings.size = 1;
        i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
        i2c_settings.reg_setting[0].delay = 0;
        i2c_settings.delay = 1;

        rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings);
        if(rc < 0)
            CAM_ERR(CAM_SENSOR, "Write ov680 reg  failed. ID:%d, addr:0x%x",s_ctrl->io_master_info.cci_client->sid,i2c_settings.reg_setting[0].reg_addr);

        i2c_settings.reg_setting[0].reg_addr = 0x6b23;
        i2c_settings.reg_setting[0].reg_data = regs_in->reg_data;
        i2c_settings.reg_setting[0].data_mask = 0;
        i2c_settings.size = 1;
        i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
        i2c_settings.reg_setting[0].delay = 0;
        i2c_settings.delay = 1;

        rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings);
        if(rc < 0)
            CAM_ERR(CAM_SENSOR, "Write ov680 reg  failed. ID:%d, addr:0x%x",s_ctrl->io_master_info.cci_client->sid,i2c_settings.reg_setting[0].reg_addr);


        i2c_settings.reg_setting[0].reg_addr = 0x6b24;
        i2c_settings.reg_setting[0].reg_data = 0x10;
        i2c_settings.reg_setting[0].data_mask = 0;
        i2c_settings.size = 1;
        i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
        i2c_settings.reg_setting[0].delay = 0;
        i2c_settings.delay = 1;

        rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings);
        if(rc < 0)
            CAM_ERR(CAM_SENSOR, "Write ov680 reg  failed. ID:%d, addr:0x%x",s_ctrl->io_master_info.cci_client->sid,i2c_settings.reg_setting[0].reg_addr);

    }
}

static void cam_ov680_write_single_cam_reg(struct cam_sensor_ctrl_t *s_ctrl,  int ctrl_value)
{
    struct cam_sensor_i2c_reg_setting i2c_settings;
    struct cam_sensor_i2c_reg_array reg_setting[1];
    int32_t rc = 0;
    uint32_t reg_data;
    uint16_t camid;


	if(0x1== (0x1&ctrl_value))
	{
		camid = 0;
	}
	else if(0x2== (0x2&ctrl_value))
	{
		camid = 1;
	}
	else if(0x4== (0x4&ctrl_value))
	{
		camid = 2;
	}
	else
	{
		camid = 0;
	}

	CAM_INFO(CAM_SENSOR,"sophia camid = %d",camid);

	switch(camid){
		case 0:
			reg_data = 0x55;
			break;
		case 1:
			reg_data = 0x00;
			break;
		case 2:
			reg_data = 0xff;
			break;
		default:
			reg_data = 0x55;
			break;
	}

        s_ctrl->io_master_info.cci_client->sid = 0x35;
        i2c_settings.reg_setting = reg_setting;


        i2c_settings.reg_setting[0].reg_addr = 0x6098;
        i2c_settings.reg_setting[0].reg_data = reg_data;
        i2c_settings.reg_setting[0].data_mask = 0;
        i2c_settings.size = 1;
        i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
        i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
        i2c_settings.reg_setting[0].delay = 0;
        i2c_settings.delay = 1;

        rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings);
        if(rc < 0)
            CAM_ERR(CAM_SENSOR, "Write ov680 reg  failed. ID:%d, addr:0x%x",s_ctrl->io_master_info.cci_client->sid,i2c_settings.reg_setting[0].reg_addr);
}
/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--end*/


static void cam_set_bit(uint16_t camera_sid)
{
	switch(camera_sid){
		case 113:
		camera_streamoff_status |= (1<<0);

		break;
		case 114:
		camera_streamoff_status |= (1<<1);
		break;

		case 115:
		camera_streamoff_status  |= (1<<2);
		break;

		case 116:
		camera_streamoff_status  |= (1<<3);
		break;

		default:
			break;
	}
	CAM_INFO(CAM_SENSOR,"sophia cam_set_bit %d sid %d",camera_streamoff_status,camera_sid);
}
static void cam_clear_bit(uint16_t camera_sid)
{
	switch(camera_sid){
		case 113:
		camera_streamoff_status&= ~(1<<0);

		break;
		case 114:
		camera_streamoff_status &= ~(1<<1);
		break;

		case 115:
		camera_streamoff_status&= ~(1<<2);
		break;

		case 116:
		camera_streamoff_status&= ~(1<<3);
		break;

		default:
			break;
	}
	CAM_INFO(CAM_SENSOR,"sophia cam_clear_bit %d sid %d",camera_streamoff_status,camera_sid);
}

int cam_sensor_is_normal_camera(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct camera_io_master *io_master_info;

	if (s_ctrl == NULL) {
		CAM_ERR(CAM_SENSOR, "Invalid args");
		return -EINVAL;
	}

	io_master_info = &s_ctrl->io_master_info;
	if(io_master_info->cci_client->sid == 0x1a || io_master_info->cci_client->sid == 0x10)
	{
		return 1;
	}

	if(io_master_info->cci_client->sid == 0x35)//ov680
	{
		return 1;
	}
	return 0;
}
static void cam_sensor_update_req_mgr(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_req_mgr_add_request add_req;

	add_req.link_hdl = s_ctrl->bridge_intf.link_hdl;
	add_req.req_id = csl_packet->header.request_id;
	CAM_DBG(CAM_SENSOR, " Rxed Req Id: %lld",
		csl_packet->header.request_id);
	add_req.dev_hdl = s_ctrl->bridge_intf.device_hdl;
	add_req.skip_before_applying = 0;
	if (s_ctrl->bridge_intf.crm_cb &&
		s_ctrl->bridge_intf.crm_cb->add_req)
		s_ctrl->bridge_intf.crm_cb->add_req(&add_req);

	CAM_DBG(CAM_SENSOR, " add req to req mgr: %lld",
			add_req.req_id);
}

static void cam_sensor_release_stream_rsc(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int rc;

	i2c_set = &(s_ctrl->i2c_data.streamoff_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamoff settings");
	}

	i2c_set = &(s_ctrl->i2c_data.streamon_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamon settings");
	}
}

static void cam_sensor_free_power_reg_rsc(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int rc;

	i2c_set = &(s_ctrl->i2c_data.poweron_reg_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting PowerOnReg settings");
	}

	i2c_set = &(s_ctrl->i2c_data.poweroff_reg_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting PowerOffReg settings");
	}
}

static void cam_sensor_release_per_frame_resource(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int i, rc;

	if (s_ctrl->i2c_data.per_frame != NULL) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(s_ctrl->i2c_data.per_frame[i]);
			if (i2c_set->is_settings_valid == 1) {
				i2c_set->is_settings_valid = -1;
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
	}
}

static int32_t cam_sensor_i2c_pkt_parse(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int32_t rc = 0;
	uintptr_t generic_ptr;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct cam_buf_io_cfg *io_cfg = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	size_t len_of_buff = 0;
	size_t remain_len = 0;
	uint32_t *offset = NULL;
	struct cam_config_dev_cmd config;
	struct i2c_data_settings *i2c_data = NULL;

	ioctl_ctrl = (struct cam_control *)arg;

	if (ioctl_ctrl->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SENSOR, "Invalid Handle Type");
		return -EINVAL;
	}

	if (copy_from_user(&config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config)))
		return -EFAULT;

	rc = cam_mem_get_cpu_buf(
		config.packet_handle,
		&generic_ptr,
		&len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed in getting the packet: %d", rc);
		return rc;
	}

	remain_len = len_of_buff;
	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_SENSOR,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto end;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)(generic_ptr +
		(uint32_t)config.offset);

	if ((csl_packet == NULL) || cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_SENSOR, "Invalid packet params");
		rc = -EINVAL;
		goto end;

	}

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG &&
		csl_packet->header.request_id <= s_ctrl->last_flush_req
		&& s_ctrl->last_flush_req != 0) {
		CAM_ERR(CAM_SENSOR,
			"reject request %lld, last request to flush %u",
			csl_packet->header.request_id, s_ctrl->last_flush_req);
		rc = -EINVAL;
		goto end;
	}

	if (csl_packet->header.request_id > s_ctrl->last_flush_req)
		s_ctrl->last_flush_req = 0;

	i2c_data = &(s_ctrl->i2c_data);
	CAM_DBG(CAM_SENSOR, "Header OpCode: %d", csl_packet->header.op_code);
	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
		i2c_reg_settings = &i2c_data->init_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
		i2c_reg_settings = &i2c_data->config_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
		if (s_ctrl->streamon_count > 0)
			goto end;

		s_ctrl->streamon_count = s_ctrl->streamon_count + 1;
		i2c_reg_settings = &i2c_data->streamon_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
		if (s_ctrl->streamoff_count > 0)
			goto end;

		s_ctrl->streamoff_count = s_ctrl->streamoff_count + 1;
		i2c_reg_settings = &i2c_data->streamoff_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_READ: {
		i2c_reg_settings = &(i2c_data->read_settings);
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;

		CAM_DBG(CAM_SENSOR, "number of IO configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_SENSOR, "No I/O configs to process");
			goto end;
		}

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_SENSOR, "I/O config is invalid(NULL)");
			goto end;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed Update packets without linking");
			goto end;
		}

		i2c_reg_settings =
			&i2c_data->per_frame[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		CAM_DBG(CAM_SENSOR, "Received Packet: %lld req: %lld",
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY,
			csl_packet->header.request_id);
		if (i2c_reg_settings->is_settings_valid == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already some pkt in offset req : %lld",
				csl_packet->header.request_id);
			/*
			 * Update req mgr even in case of failure.
			 * This will help not to wait indefinitely
			 * and freeze. If this log is triggered then
			 * fix it.
			 */
			cam_sensor_update_req_mgr(s_ctrl, csl_packet);
			goto end;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_NOP: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed NOP packets without linking");
			goto end;
		}

		cam_sensor_update_req_mgr(s_ctrl, csl_packet);
		goto end;
	}
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Packet Header");
		rc = -EINVAL;
		goto end;
	}

	offset = (uint32_t *)&csl_packet->payload;
	offset += csl_packet->cmd_buf_offset / 4;
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
			i2c_reg_settings, cmd_desc, 1, io_cfg);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Fail parsing I2C Pkt: %d", rc);
		goto end;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) ==
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE) {
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		cam_sensor_update_req_mgr(s_ctrl, csl_packet);
	}

end:
	return rc;
}

static int32_t cam_sensor_restore_slave_info(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	switch (s_ctrl->io_master_info.master_type) {
	case CCI_MASTER:
		s_ctrl->io_master_info.cci_client->sid =
			(s_ctrl->sensordata->slave_info.sensor_slave_addr >> 1);
		s_ctrl->io_master_info.cci_client->i2c_freq_mode =
			s_ctrl->sensordata->slave_info.i2c_freq_mode;
		break;

	case I2C_MASTER:
		s_ctrl->io_master_info.client->addr =
			 s_ctrl->sensordata->slave_info.sensor_slave_addr;
		break;

	case SPI_MASTER:
		break;

	default:
		CAM_ERR(CAM_SENSOR, "Invalid master type: %d",
				s_ctrl->io_master_info.master_type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t cam_sensor_update_i2c_info(struct cam_cmd_i2c_info *i2c_info,
	struct cam_sensor_ctrl_t *s_ctrl,
	bool isInit)
{
	int32_t rc = 0;
	struct cam_sensor_cci_client   *cci_client = NULL;

	switch (s_ctrl->io_master_info.master_type) {
	case CCI_MASTER:
		cci_client = s_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_SENSOR, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
		cci_client->sid = i2c_info->slave_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
		CAM_DBG(CAM_SENSOR, " Master: %d sid: 0x%x freq_mode: %d",
			cci_client->cci_i2c_master, i2c_info->slave_addr,
			i2c_info->i2c_freq_mode);
		break;

	case I2C_MASTER:
		s_ctrl->io_master_info.client->addr = i2c_info->slave_addr;
		break;

	case SPI_MASTER:
		break;

	default:
		CAM_ERR(CAM_SENSOR, "Invalid master type: %d",
			s_ctrl->io_master_info.master_type);
		rc = -EINVAL;
		break;
	}

	if (isInit) {
		s_ctrl->sensordata->slave_info.sensor_slave_addr =
			i2c_info->slave_addr;
		s_ctrl->sensordata->slave_info.i2c_freq_mode =
			i2c_info->i2c_freq_mode;
	}

	return rc;
}

static int32_t cam_sensor_i2c_modes_util(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;
	//uint32_t j;
	struct camera_io_master *io_master_info;

	if (s_ctrl == NULL) {
		CAM_ERR(CAM_SENSOR, "Invalid args");
		return -EINVAL;
	}

	io_master_info = &s_ctrl->io_master_info;

	if(cam_sensor_is_normal_camera(s_ctrl) == 1)
	{
	/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--start*/
		/*for ov680 strobe always on*/
		if((io_master_info->cci_client->sid == 0x35)&&(i2c_list->i2c_settings.size == 1)&&(i2c_list->i2c_settings.reg_setting[0].reg_addr == 0xffff))
		{
			if(0x8 == (0x8&ov680_ctrl_value))
			{
				cam_ov680_write_regs(s_ctrl, ov680_strobe_regs, STROBE_REG_COUNT);
				CAM_INFO(CAM_SENSOR,"sophia ov680 strobe always: on reg_addr 0x%x sid %d",i2c_list->i2c_settings.reg_setting[0].reg_addr,io_master_info->cci_client->sid);
			}
			else if(0x10 == (0x10&ov680_ctrl_value))/*for ov680 alternant*/
			{
				cam_ov680_write_regs(s_ctrl, ov680_alternant_regs, ALTERNANT_REG_COUNT);
				CAM_INFO(CAM_SENSOR,"sophia ov680 alternant :reg_addr 0x%x sid %d",i2c_list->i2c_settings.reg_setting[0].reg_addr,io_master_info->cci_client->sid);
			}
			return rc;

		}
		/*for ov680 400x400*/
		else if((io_master_info->cci_client->sid == 0x35)&&(i2c_list->i2c_settings.size == 1)&&(i2c_list->i2c_settings.reg_setting[0].reg_addr == 0xeeee))
		{
			cam_ov680_write_single_cam_reg(s_ctrl, ov680_ctrl_value);
			CAM_INFO(CAM_SENSOR,"sophia reg_addr 0x%x sid %d",i2c_list->i2c_settings.reg_setting[0].reg_addr,io_master_info->cci_client->sid);
			return rc;

		}
	/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--end*/

	//CAM_INFO(CAM_SENSOR,"sophia  size %d,addr 0x%x reg_data 0x%x sid %d",i2c_list->i2c_settings.size,i2c_list->i2c_settings.reg_setting[0].reg_addr,i2c_list->i2c_settings.reg_setting[0].reg_data,io_master_info->cci_client->sid);
}
else
{
	#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
	CAM_INFO(CAM_SENSOR,"sophia regwirte size %d,addr 0x%x reg_data 0x%x sid %d",i2c_list->i2c_settings.size,i2c_list->i2c_settings.reg_setting[0].reg_addr,i2c_list->i2c_settings.reg_setting[0].reg_data,io_master_info->cci_client->sid);
	if((i2c_list->i2c_settings.size == 1)&&(i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x100)&&(i2c_list->i2c_settings.reg_setting[0].reg_data == 0x01))
	{
		cam_clear_bit(io_master_info->cci_client->sid);
	}
	else if((i2c_list->i2c_settings.size == 1)&&(i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x100)&&(i2c_list->i2c_settings.reg_setting[0].reg_data == 0x00))
	{
		cam_set_bit(io_master_info->cci_client->sid);

		//CAM_INFO(CAM_SENSOR,"sophia off out camera_streamoff_status %d sid %d",camera_streamoff_status,io_master_info->cci_client->sid);
		msleep(200);
		//CAM_INFO(CAM_SENSOR,"sophia i2c_list->op_code %d sid %d",i2c_list->op_code,io_master_info->cci_client->sid);
	}

	  return rc;

 	#endif
}


	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to random write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			0);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to seq write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to burst write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
		size = i2c_list->i2c_settings.size;
		for (i = 0; i < size; i++) {
			rc = camera_io_dev_poll(
			io_master_info,
			i2c_list->i2c_settings.reg_setting[i].reg_addr,
			i2c_list->i2c_settings.reg_setting[i].reg_data,
			i2c_list->i2c_settings.reg_setting[i].data_mask,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type,
			i2c_list->i2c_settings.reg_setting[i].delay);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_SET_I2C_INFO) {
		rc = cam_sensor_update_i2c_info(&i2c_list->slave_info,
			s_ctrl,
			false);
	} else if ((i2c_list->op_code == CAM_SENSOR_I2C_READ_RANDOM) ||
		(i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ)) {
		rc = cam_sensor_i2c_read_data(
			&s_ctrl->i2c_data.read_settings,
			&s_ctrl->io_master_info);
	}
	if((io_master_info->cci_client->sid == 0x35)&&(i2c_list->i2c_settings.size == 1)&&
	        ((i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x6103)||(i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x8c21)))
	{
		msleep(1);
		CAM_INFO(CAM_SENSOR,"sophia reg_addr 0x%x sid %d",i2c_list->i2c_settings.reg_setting[0].reg_addr,io_master_info->cci_client->sid);

	}
	    if((s_ctrl->io_master_info.cci_client->sid == 0x35)&&(i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x6b0e))
	    {
			msleep(80);
			CAM_INFO(CAM_SENSOR,"sophia sleep =======================================80ms");
		}

	return rc;
}

int32_t cam_sensor_update_slave_info(struct cam_cmd_probe *probe_info,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	s_ctrl->sensordata->slave_info.sensor_id_reg_addr =
		probe_info->reg_addr;
	s_ctrl->sensordata->slave_info.sensor_id =
		probe_info->expected_data;
	s_ctrl->sensordata->slave_info.sensor_id_mask =
		probe_info->data_mask;
	/* Userspace passes the pipeline delay in reserved field */
	s_ctrl->pipeline_delay =
		probe_info->reserved;

	s_ctrl->sensor_probe_addr_type =  probe_info->addr_type;
	s_ctrl->sensor_probe_data_type =  probe_info->data_type;
	CAM_DBG(CAM_SENSOR,
		"Sensor Addr: 0x%x sensor_id: 0x%x sensor_mask: 0x%x sensor_pipeline_delay:0x%x",
		s_ctrl->sensordata->slave_info.sensor_id_reg_addr,
		s_ctrl->sensordata->slave_info.sensor_id,
		s_ctrl->sensordata->slave_info.sensor_id_mask,
		s_ctrl->pipeline_delay);
	return rc;
}

int32_t cam_handle_cmd_buffers_for_probe(void *cmd_buf,
	struct cam_sensor_ctrl_t *s_ctrl,
	int32_t cmd_buf_num, uint32_t cmd_buf_length, size_t remain_len,
	struct cam_cmd_buf_desc   *cmd_desc)
{
	int32_t rc = 0;

	switch (cmd_buf_num) {
	case 0: {
		struct cam_cmd_i2c_info *i2c_info = NULL;
		struct cam_cmd_probe *probe_info;

		if (remain_len <
			(sizeof(struct cam_cmd_i2c_info) +
			sizeof(struct cam_cmd_probe))) {
			CAM_ERR(CAM_SENSOR,
				"not enough buffer for cam_cmd_i2c_info");
			return -EINVAL;
		}
		i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
		rc = cam_sensor_update_i2c_info(i2c_info, s_ctrl, true);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed in Updating the i2c Info");
			return rc;
		}
		probe_info = (struct cam_cmd_probe *)
			(cmd_buf + sizeof(struct cam_cmd_i2c_info));
		rc = cam_sensor_update_slave_info(probe_info, s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Updating the slave Info");
			return rc;
		}
		cmd_buf = probe_info;
	}
		break;
	case 1: {
		rc = cam_sensor_update_power_settings(cmd_buf,
			cmd_buf_length, &s_ctrl->sensordata->power_info,
			remain_len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating power settings");
			return rc;
		}
	}
		break;
	case 2: {
		struct i2c_settings_array *i2c_reg_settings = NULL;
		struct i2c_data_settings *i2c_data = NULL;
		struct cam_buf_io_cfg *io_cfg = NULL;

		CAM_DBG(CAM_SENSOR, "poweron_reg_settings");
		i2c_data = &(s_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->poweron_reg_settings;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
				i2c_reg_settings, cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating power register settings");
			return rc;
		}
	}
		break;
	case 3: {
		struct i2c_settings_array *i2c_reg_settings = NULL;
		struct i2c_data_settings *i2c_data = NULL;
		struct cam_buf_io_cfg *io_cfg = NULL;

		CAM_DBG(CAM_SENSOR, "poweroff_reg_settings");
		i2c_data = &(s_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->poweroff_reg_settings;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
				i2c_reg_settings, cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating power register settings");
			return rc;
		}
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid command buffer");
		break;
	}
	return rc;
}

int32_t cam_handle_mem_ptr(uint64_t handle, struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, i;
	uint32_t *cmd_buf;
	void *ptr;
	size_t len;
	struct cam_packet *pkt = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	uintptr_t cmd_buf1 = 0;
	uintptr_t packet = 0;
	size_t    remain_len = 0;

	rc = cam_mem_get_cpu_buf(handle,
		&packet, &len);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to get the command Buffer");
		return -EINVAL;
	}

	pkt = (struct cam_packet *)packet;
	if (pkt == NULL) {
		CAM_ERR(CAM_SENSOR, "packet pos is invalid");
		rc = -EINVAL;
		goto end;
	}

	if ((len < sizeof(struct cam_packet)) ||
		(pkt->cmd_buf_offset >= (len - sizeof(struct cam_packet)))) {
		CAM_ERR(CAM_SENSOR, "Not enough buf provided");
		rc = -EINVAL;
		goto end;
	}

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *)&pkt->payload + pkt->cmd_buf_offset/4);
	if (cmd_desc == NULL) {
		CAM_ERR(CAM_SENSOR, "command descriptor pos is invalid");
		rc = -EINVAL;
		goto end;
	}
	if (pkt->num_cmd_buf < 2) {
		CAM_ERR(CAM_SENSOR, "Expected More Command Buffers : %d",
			 pkt->num_cmd_buf);
		rc = -EINVAL;
		goto end;
	}

	for (i = 0; i < pkt->num_cmd_buf; i++) {
		if (!(cmd_desc[i].length))
			continue;
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&cmd_buf1, &len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the command Buffer Header");
			goto end;
		}
		if (cmd_desc[i].offset >= len) {
			CAM_ERR(CAM_SENSOR,
				"offset past length of buffer");
			rc = -EINVAL;
			goto end;
		}
		remain_len = len - cmd_desc[i].offset;
		if (cmd_desc[i].length > remain_len) {
			CAM_ERR(CAM_SENSOR,
				"Not enough buffer provided for cmd");
			rc = -EINVAL;
			goto end;
		}
		cmd_buf = (uint32_t *)cmd_buf1;
		cmd_buf += cmd_desc[i].offset/4;
		ptr = (void *) cmd_buf;

		rc = cam_handle_cmd_buffers_for_probe(ptr, s_ctrl,
			i, cmd_desc[i].length, remain_len, &cmd_desc[i]);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the command Buffer Header");
			goto end;
		}
	}

end:
	return rc;
}

void cam_sensor_query_cap(struct cam_sensor_ctrl_t *s_ctrl,
	struct  cam_sensor_query_cap *query_cap)
{
	query_cap->pos_roll = s_ctrl->sensordata->pos_roll;
	query_cap->pos_pitch = s_ctrl->sensordata->pos_pitch;
	query_cap->pos_yaw = s_ctrl->sensordata->pos_yaw;
	query_cap->secure_camera = 0;
	query_cap->actuator_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_ACTUATOR];
	query_cap->csiphy_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY];
	query_cap->eeprom_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_EEPROM];
	query_cap->flash_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_LED_FLASH];
	query_cap->ois_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_OIS];
	query_cap->slot_info =
		s_ctrl->soc_info.index;
}

static uint16_t cam_sensor_id_by_mask(struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t chipid)
{
	uint16_t sensor_id = (uint16_t)(chipid & 0xFFFF);
	int16_t sensor_id_mask = s_ctrl->sensordata->slave_info.sensor_id_mask;

	if (!sensor_id_mask)
		sensor_id_mask = ~sensor_id_mask;

	sensor_id &= sensor_id_mask;
	sensor_id_mask &= -sensor_id_mask;
	sensor_id_mask -= 1;
	while (sensor_id_mask) {
		sensor_id_mask >>= 1;
		sensor_id >>= 1;
	}
	return sensor_id;
}

void cam_sensor_shutdown(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info =
		&s_ctrl->sensordata->power_info;
	int rc = 0;

	if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) &&
		(s_ctrl->is_probe_succeed == 0))
		return;

	cam_sensor_release_stream_rsc(s_ctrl);
	cam_sensor_release_per_frame_resource(s_ctrl);

	if (s_ctrl->sensor_state != CAM_SENSOR_INIT)
		cam_sensor_power_down(s_ctrl);

	if (s_ctrl->bridge_intf.device_hdl != -1) {
		rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"dhdl already destroyed: rc = %d", rc);
	}

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.session_hdl = -1;
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;
	s_ctrl->streamon_count = 0;
	s_ctrl->streamoff_count = 0;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->last_flush_req = 0;
	s_ctrl->sensor_state = CAM_SENSOR_INIT;
}
#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
static ssize_t sixdof_Fsin_boottime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "%lld\n", Fsin_boot_timestamp);
}
static DEVICE_ATTR( sixdof_Fsin_boottime, 0664, sixdof_Fsin_boottime_show, NULL);
static ssize_t sixdof_streamoff_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "%d\n", camera_streamoff_status);
}
static DEVICE_ATTR( sixdof_streamoff_status, 0664, sixdof_streamoff_status_show, NULL);

#endif

/*Modify by PICO_Driver sophia.wang for bug55603 : add nodes to judge if eyetracking 6dof exist start*/
uint32_t camera_exist = 0;

/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--start*/
static ssize_t ov680_ctrl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "%d\n", ov680_ctrl_value);
}
static ssize_t ov680_ctrl_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
		int value = 0;
		if(sscanf(buf, "%x",  &value) != 1) {
			return -1;
		}

		CAM_DBG(CAM_SENSOR, "set ov680 ctrl value:%d",value);
		ov680_ctrl_value = value;
		return count;
}
static DEVICE_ATTR( ov680_ctrl, 0664, ov680_ctrl_show, ov680_ctrl_store);

static struct attribute *ov680_attributes[] = {
	&dev_attr_ov680_ctrl.attr,
	NULL
};
static struct attribute_group ov680_attribute_group = {
	.attrs = ov680_attributes
};
/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--end*/

#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
static struct attribute *sixdof_four_cameras_attributes[] = {
	&dev_attr_sixdof_Fsin_boottime.attr,
	&dev_attr_sixdof_streamoff_status.attr,
	NULL
};
static struct attribute_group sixdof_four_cameras_attribute_group = {
	.attrs = sixdof_four_cameras_attributes
};
#endif

#ifdef __CAMERA_EDITOR__
void cam_sensor_exist_status(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_camera_slave_info *slave_info;

	slave_info = &(s_ctrl->sensordata->slave_info);

	/*6dof*/
	if((s_ctrl->soc_info.index == 2)&&(slave_info->sensor_id == 0x7750))
	{
        pico_camera_set_camera_bit(SIXDOF_TL);
	}
	else if((s_ctrl->soc_info.index == 3)&&(slave_info->sensor_id == 0x7750))
	{
        pico_camera_set_camera_bit(SIXDOF_BR);
	}
	else if((s_ctrl->soc_info.index == 0)&&(slave_info->sensor_id == 0x7750))
	{
        pico_camera_set_camera_bit(SIXDOF_TR);
	}
	else if((s_ctrl->soc_info.index == 1)&&(slave_info->sensor_id == 0x7750))
	{
        pico_camera_set_camera_bit(SIXDOF_BL);
	}
	CAM_INFO(CAM_SENSOR, "camera_exist status:0x%x\n",camera_exist);

}
/*Modify by PICO_Driver sophia.wang for bug55603 : add nodes to judge if eyetracking 6dof exist end*/
#endif

int cam_sensor_match_id(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t chipid = 0;
	struct cam_camera_slave_info *slave_info;

#ifdef __CAMERA_EDITOR__
	#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
	if(s_ctrl->soc_info.index == 2)
	{
		 sysfs_remove_group(&s_ctrl->pdev->dev.kobj, &sixdof_four_cameras_attribute_group);

		if (sysfs_create_group(&s_ctrl->pdev->dev.kobj, &sixdof_four_cameras_attribute_group) < 0) {
					CAM_ERR(CAM_SENSOR, "%s: create sixdof_four_cameras_attribute_group system file node failure!\n", __func__);
		}
		Fsin_irq_register = 0;//when do recovey Fsin_irq_register should be clear
	}
	#endif


	/*Modify by PICO_Driver sophia.wang for bug1178: bringup eyetracking camera in xr2 cv3 start*/
	//0x75 is (0xEA >> 1) 0x76 is (0xEC >> 1)
	if((s_ctrl->io_master_info.cci_client->sid == 0x75)||(s_ctrl->io_master_info.cci_client->sid == 0x76))
	{
		CAM_INFO(CAM_SENSOR, "ET camera");
	}
	else if(s_ctrl->io_master_info.cci_client->sid == 0x1a || s_ctrl->io_master_info.cci_client->sid == 0x10)
	{
		CAM_INFO(CAM_SENSOR, "ET imx476 camera");
	}
	else if(s_ctrl->io_master_info.cci_client->sid == 0x35)
	{
		CAM_INFO(CAM_SENSOR, "ov680 camera");
		/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--start*/
		/*for ov680 led and single camera*/
		sysfs_remove_group(&s_ctrl->pdev->dev.kobj, &ov680_attribute_group);
		if (sysfs_create_group(&s_ctrl->pdev->dev.kobj, &ov680_attribute_group) < 0) {
			CAM_ERR(CAM_SENSOR, "%s: create ov680_attribute_group system file node failure!\n", __func__);
		}
		/*added by florence.wang for Ticket 3101017 - ov680 led alwayson and size 400x400--end*/
	}
	else
	{
		cam_sensor_exist_status(s_ctrl);
		return rc;
	}
	/*Modify by PICO_Driver sophia.wang for bug1178: bringup eyetracking camera in xr2 cv3 end*/
#endif

	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " failed: %pK",
			 slave_info);
		return -EINVAL;
	}

	rc = camera_io_dev_read(
		&(s_ctrl->io_master_info),
		slave_info->sensor_id_reg_addr,
		&chipid,
		s_ctrl->sensor_probe_addr_type,
		s_ctrl->sensor_probe_data_type);

	CAM_ERR(CAM_SENSOR, "read id: 0x%x expected id 0x%x:",
		chipid, slave_info->sensor_id);

	if (cam_sensor_id_by_mask(s_ctrl, chipid) != slave_info->sensor_id) {
		CAM_WARN(CAM_SENSOR, "read id: 0x%x expected id 0x%x:",
				chipid, slave_info->sensor_id);
		return -ENODEV;
	}
#ifdef __CAMERA_EDITOR__
	cam_sensor_exist_status(s_ctrl);/*Modify by PICO_Driver sophia.wang for bug55603 : add nodes to judge if eyetracking 6dof exist*/
#endif
	return rc;
}

#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
static irqreturn_t sixdof_4cam_Fsin_irq(int irq, void *_dev)
{
	struct timespec64 ts;

	//CAM_INFO(CAM_SENSOR,"sophia sixdof_4cam_Fsin_irq !");
	get_monotonic_boottime64(&ts);
	Fsin_boot_timestamp =
				(uint64_t)((ts.tv_sec * 1000000000) +
				ts.tv_nsec);

	//CAM_INFO(CAM_SENSOR,"sophia Fsin_boot_timestamp %lld",Fsin_boot_timestamp);
	return IRQ_HANDLED;
}

int cam_sensor_register_irq(struct cam_sensor_ctrl_t *s_ctrl)
{
	int retval = 0;

	if(cam_sensor_is_normal_camera(s_ctrl) == 1){
	        return retval;
	}

	CAM_INFO(CAM_SENSOR,"sophia	beofore register Fsin_gpio Fsin_irq_register = %d",Fsin_irq_register);

	if(Fsin_irq_register == 0)
	{
		//CAM_INFO(CAM_SENSOR,"sophia	register Fsin_gpio");

		retval = gpio_request(Fsin_gpio, "Fsin_4cam Fsin_gpio");
		 if (retval < 0) {
			 CAM_ERR(CAM_SENSOR,"sophia  cannot claim Fsin_gpio");
		 }

		 retval = gpio_direction_input(Fsin_gpio);
		 if (retval < 0) {
			 CAM_ERR(CAM_SENSOR,"sophia    unable to set the Fsin_gpio direction");
		 }

		 Fsin_gpio_irq = gpio_to_irq(Fsin_gpio);
		 if (Fsin_gpio_irq < 0) {
		  CAM_ERR(CAM_SENSOR,"sophia  no irq for Fsin_gpio");
			 return retval;
		 }
		//IRQF_TRIGGER_FALLING,IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW |IRQF_TRIGGER_RISING
		 retval = request_irq(Fsin_gpio_irq, sixdof_4cam_Fsin_irq,
					  IRQF_TRIGGER_RISING,
					  "Fsin Fsin_gpio", &(s_ctrl->io_master_info.client->dev));

		 if (retval != 0) {
			 CAM_ERR(CAM_SENSOR,"can't get irq %d, err %d",Fsin_gpio_irq, retval);
			 }

	}
	Fsin_irq_register++;
	return retval;

}

int cam_sensor_unregister_irq(struct cam_sensor_ctrl_t *s_ctrl)
{
	if(cam_sensor_is_normal_camera(s_ctrl) == 1){
		return 0;
	}

	CAM_INFO(CAM_SENSOR,"sophia	beofore unregister Fsin_gpio Fsin_irq_register = %d",Fsin_irq_register);
	Fsin_irq_register--;
	if(Fsin_irq_register == 0)
	{
		if (Fsin_gpio_irq)
		{
			//CAM_INFO(CAM_SENSOR,"sophia do unregister Fsin_gpio");
			free_irq(Fsin_gpio_irq,&(s_ctrl->io_master_info.client->dev));
			gpio_free(Fsin_gpio);
		}
	}
	return 0;
}
#endif
int32_t cam_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int rc = 0, pkt_opcode = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_sensor_power_ctrl_t *power_info =
		&s_ctrl->sensordata->power_info;
	if (!s_ctrl || !arg) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
		return -EINVAL;
	}

	if (cmd->op_code != CAM_SENSOR_PROBE_CMD) {
		if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
			CAM_ERR(CAM_SENSOR, "Invalid handle type: %d",
				cmd->handle_type);
			return -EINVAL;
		}
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	switch (cmd->op_code) {
	case CAM_SENSOR_PROBE_CMD: {
		//CAM_INFO(CAM_SENSOR, "sophia ========= CAM_SENSOR_PROBE_CMD");
		if (s_ctrl->is_probe_succeed == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already Sensor Probed in the slot");
			break;
		}

		if (cmd->handle_type ==
			CAM_HANDLE_MEM_HANDLE) {
			rc = cam_handle_mem_ptr(cmd->handle, s_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "Get Buffer Handle Failed");
				goto release_mutex;
			}
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid Command Type: %d",
				 cmd->handle_type);
			rc = -EINVAL;
			goto release_mutex;
		}

		/* Parse and fill vreg params for powerup settings */
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_setting,
			s_ctrl->sensordata->power_info.power_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PUP rc %d",
				 rc);
			goto free_power_settings;
		}

		/* Parse and fill vreg params for powerdown settings*/
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_down_setting,
			s_ctrl->sensordata->power_info.power_down_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PDOWN rc %d",
				 rc);
			goto free_power_settings;
		}

		/* Power up and probe sensor */
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "power up failed");
			goto free_power_settings;
		}
		if (s_ctrl->i2c_data.poweron_reg_settings.is_settings_valid) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_POWERON_REG);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "PowerOn REG_WR failed");
				goto free_power_settings;
			}
		}

		/* Match sensor ID */
		rc = cam_sensor_match_id(s_ctrl);
		if (rc < 0) {
			cam_sensor_power_down(s_ctrl);
			msleep(20);
			goto free_power_settings;
		}

		if (s_ctrl->i2c_data.poweroff_reg_settings.is_settings_valid) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_POWEROFF_REG);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "PowerOff REG_WR failed");
				goto free_power_settings;
			}
		}

		CAM_INFO(CAM_SENSOR,
			"Probe success,slot:%d,slave_addr:0x%x,sensor_id:0x%x",
			s_ctrl->soc_info.index,
			s_ctrl->sensordata->slave_info.sensor_slave_addr,
			s_ctrl->sensordata->slave_info.sensor_id);

		cam_sensor_free_power_reg_rsc(s_ctrl);
		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "fail in Sensor Power Down");
			goto free_power_settings;
		}
		/*
		 * Set probe succeeded flag to 1 so that no other camera shall
		 * probed on this slot
		 */
		s_ctrl->is_probe_succeed = 1;
		s_ctrl->sensor_state = CAM_SENSOR_INIT;
	}
		break;
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev sensor_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		//CAM_INFO(CAM_SENSOR, "sophia ========= CAM_ACQUIRE_DEV");
		if ((s_ctrl->is_probe_succeed == 0) ||
			(s_ctrl->sensor_state != CAM_SENSOR_INIT)) {
			CAM_WARN(CAM_SENSOR,
				"Not in right state to aquire %d， probe %d",
				s_ctrl->sensor_state, s_ctrl->is_probe_succeed);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_SENSOR, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&sensor_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(sensor_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed Copying from user");
			goto release_mutex;
		}

		bridge_params.session_hdl = sensor_acq_dev.session_handle;
		bridge_params.ops = &s_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = s_ctrl;

		sensor_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		s_ctrl->bridge_intf.device_hdl = sensor_acq_dev.device_handle;
		s_ctrl->bridge_intf.session_hdl = sensor_acq_dev.session_handle;

		CAM_DBG(CAM_SENSOR, "Device Handle: %d",
			sensor_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
		cam_sensor_register_irq(s_ctrl);
		#endif
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Sensor Power up failed");
			goto release_mutex;
		}

		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		s_ctrl->last_flush_req = 0;
		CAM_INFO(CAM_SENSOR,
			"CAM_ACQUIRE_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_RELEASE_DEV: {
		//CAM_INFO(CAM_SENSOR, "sophia  ========= CAM_RELEASE_DEV");
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to release : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_SENSOR,
				"Device [%d] still active on link 0x%x",
				s_ctrl->sensor_state,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Sensor Power Down failed");
			goto release_mutex;
		}

		#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
		cam_sensor_unregister_irq(s_ctrl);
		#endif

		cam_sensor_release_per_frame_resource(s_ctrl);
		cam_sensor_release_stream_rsc(s_ctrl);
		if (s_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_SENSOR,
				"Invalid Handles: link hdl: %d device hdl: %d",
				s_ctrl->bridge_intf.device_hdl,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed in destroying the device hdl");
		s_ctrl->bridge_intf.device_hdl = -1;
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.session_hdl = -1;

		s_ctrl->sensor_state = CAM_SENSOR_INIT;
		CAM_INFO(CAM_SENSOR,
			"CAM_RELEASE_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
		s_ctrl->streamon_count = 0;
		s_ctrl->streamoff_count = 0;
		s_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_QUERY_CAP: {
		struct  cam_sensor_query_cap sensor_cap;

		//CAM_INFO(CAM_SENSOR, "sophia  CAM_QUERY_CAP");

		cam_sensor_query_cap(s_ctrl, &sensor_cap);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_cap, sizeof(struct  cam_sensor_query_cap))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_START_DEV: {
		//CAM_INFO(CAM_SENSOR, "sophia  ========= CAM_START_DEV");
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to start : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->i2c_data.streamon_settings.is_settings_valid &&
			(s_ctrl->i2c_data.streamon_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply streamon settings");
				goto release_mutex;
			}
		}
		s_ctrl->sensor_state = CAM_SENSOR_START;
		CAM_INFO(CAM_SENSOR,
			"CAM_START_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_STOP_DEV: {
		//CAM_INFO(CAM_SENSOR, "sophia ========= CAM_STOP_DEV");
		if (s_ctrl->sensor_state != CAM_SENSOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to stop : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->i2c_data.streamoff_settings.is_settings_valid &&
			(s_ctrl->i2c_data.streamoff_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
				"cannot apply streamoff settings");
			}
		}

		cam_sensor_release_per_frame_resource(s_ctrl);
		s_ctrl->last_flush_req = 0;
		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		CAM_INFO(CAM_SENSOR,
			"CAM_STOP_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_CONFIG_DEV: {
		//CAM_INFO(CAM_SENSOR, "sophia ========= CAM_CONFIG_DEV");
		rc = cam_sensor_i2c_pkt_parse(s_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed i2c pkt parse: %d", rc);
			goto release_mutex;
		}
		if (s_ctrl->i2c_data.init_settings.is_settings_valid &&
			(s_ctrl->i2c_data.init_settings.request_id == 0)) {

			pkt_opcode =
				CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG;
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				pkt_opcode);

			if ((rc == -EAGAIN) &&
			(s_ctrl->io_master_info.master_type == CCI_MASTER)) {
				/* If CCI hardware is resetting we need to wait
				 * for sometime before reapply
				 */
				CAM_WARN(CAM_SENSOR,
					"Reapplying the Init settings due to cci hw reset");
				usleep_range(1000, 1010);
				rc = cam_sensor_apply_settings(s_ctrl, 0,
					pkt_opcode);
			}
			s_ctrl->i2c_data.init_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply init settings rc= %d",
					rc);
				delete_request(&s_ctrl->i2c_data.init_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.init_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the Init settings");
				goto release_mutex;
			}
		}

		if (s_ctrl->i2c_data.config_settings.is_settings_valid &&
			(s_ctrl->i2c_data.config_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);

			s_ctrl->i2c_data.config_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply config settings");
				delete_request(
					&s_ctrl->i2c_data.config_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.config_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the config settings");
				goto release_mutex;
			}
			s_ctrl->sensor_state = CAM_SENSOR_CONFIG;
		}

		if (s_ctrl->i2c_data.read_settings.is_settings_valid) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_READ);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply read settings");
				delete_request(
					&s_ctrl->i2c_data.read_settings);
				goto release_mutex;
			}
			rc = delete_request(
				&s_ctrl->i2c_data.read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the read settings");
				goto release_mutex;
			}
		}

		CAM_DBG(CAM_SENSOR,
			"CAM_CONFIG_DEV done sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
		goto release_mutex;
	}

release_mutex:
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
	cam_sensor_free_power_reg_rsc(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!info)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(info->dev_hdl);

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	info->dev_id = CAM_REQ_MGR_DEVICE_SENSOR;
	strlcpy(info->name, CAM_SENSOR_NAME, sizeof(info->name));
	if (s_ctrl->pipeline_delay >= 1 && s_ctrl->pipeline_delay <= 3)
		info->p_delay = s_ctrl->pipeline_delay;
	else
		info->p_delay = 2;
	info->trigger = CAM_TRIGGER_POINT_SOF;

	return rc;
}

int cam_sensor_establish_link(struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!link)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(link->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&s_ctrl->cam_sensor_mutex);
	if (link->link_enable) {
		s_ctrl->bridge_intf.link_hdl = link->link_hdl;
		s_ctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&s_ctrl->cam_sensor_mutex);

	return 0;
}

int cam_sensor_power(struct v4l2_subdev *sd, int on)
{
	struct cam_sensor_ctrl_t *s_ctrl = v4l2_get_subdevdata(sd);

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if (!on && s_ctrl->sensor_state == CAM_SENSOR_START) {
		cam_sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
	}
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return 0;
}
/*added by webber.wang for Bug 55091 - [neo2.5]four 6dof camera sensors debug--start*/
#ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT

int cam_sensor_read_addr(struct cam_sensor_ctrl_t *s_ctrl,uint32_t addr,uint32_t *data)
{
	int rc = 0;
	struct cam_camera_slave_info *slave_info;

	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " failed: %pK",
			 slave_info);
		return -EINVAL;
	}
	rc = camera_io_dev_read(
		&(s_ctrl->io_master_info),
		addr,
		data, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	return rc;
}

#define CAM_COUNT 2
struct cam_sensor_slave_addr neo2_5_cam_sensor_addr[CAM_COUNT] = {
    //{2, 0x9281, 0xE0, 0xE6 },//pico-6dof_right
    //{2, 0x7750, 0xE0, 0xE2 },//pico-6dof_right
    //{3, 0x9281, 0xE0, 0xE8 },//pico_6dof_left
    //{3, 0x7750, 0xE0, 0xE4 },//pico_6dof_left
    //{4, 0x9281, 0xE0, 0xE4 },//qcom_6dof_right
    //{4, 0x7750, 0xE0, 0xE6 },//qcom_6dof_right
    //{5, 0x9281, 0xE0, 0xE2 }, //qcom_6dof_left
    //{5, 0x7750, 0xE0, 0xE8 }, //qcom_6dof_left
    {0, 0x6710, 0xE0, 0xEA },//eyetracking_left
    {1, 0x6710, 0xE0, 0xEC },//eyetracking_right
};
static void cam_change_sensor_private_slave_address(struct cam_sensor_ctrl_t *s_ctrl)
{
    struct cam_sensor_i2c_reg_setting i2c_settings1;
	struct cam_sensor_i2c_reg_array real_reg_setting1[1];
    uint32_t data = 0;
    int32_t rc = 0;
    int i = 0;
    for(i = 0;i < CAM_COUNT; i++)
    {
        if((neo2_5_cam_sensor_addr[i].index == s_ctrl->soc_info.index)
            && (neo2_5_cam_sensor_addr[i].sensor_id == s_ctrl->sensordata->slave_info.sensor_id))
        {
            i2c_settings1.reg_setting = real_reg_setting1;
            i2c_settings1.reg_setting[0].reg_addr = 0x302B;
            i2c_settings1.reg_setting[0].reg_data = neo2_5_cam_sensor_addr[i].new_slave_addr;
            i2c_settings1.reg_setting[0].data_mask = 0;
            i2c_settings1.size = 1;
            i2c_settings1.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
            i2c_settings1.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
            i2c_settings1.reg_setting[0].delay = 0;
            i2c_settings1.delay = 0;

            s_ctrl->io_master_info.cci_client->sid = (neo2_5_cam_sensor_addr[i].def_slave_addr >> 1);
            rc = camera_io_dev_write(&(s_ctrl->io_master_info),&i2c_settings1);
            if(rc < 0)
                CAM_ERR(CAM_SENSOR, "Write sensor slave addr failed. ID:%d, addr:0x%x",neo2_5_cam_sensor_addr[i].index,neo2_5_cam_sensor_addr[i].new_slave_addr);
            s_ctrl->io_master_info.cci_client->sid = (neo2_5_cam_sensor_addr[i].new_slave_addr >> 1);
            rc = cam_sensor_read_addr(s_ctrl,0x302B,&data);
            if(rc < 0)
                CAM_ERR(CAM_SENSOR, "Read sensor slave addr failed. ID:%d",neo2_5_cam_sensor_addr[i].index);
            if(neo2_5_cam_sensor_addr[i].new_slave_addr != data)
                CAM_ERR(CAM_SENSOR, "Change sensor slave addr failed, ID:%d, addr:0x%x",neo2_5_cam_sensor_addr[i].index,neo2_5_cam_sensor_addr[i].new_slave_addr);

        }
    }
}
#endif
/*added by webber.wang for Bug 55091 - [neo2.5]four 6dof camera sensors debug--end*/
int cam_sensor_power_up(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_camera_slave_info *slave_info;
	struct cam_hw_soc_info *soc_info;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!power_info || !slave_info) {
		CAM_ERR(CAM_SENSOR, "failed: %pK %pK", power_info, slave_info);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;

	if (s_ctrl->bob_pwm_switch) {
		rc = cam_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, true);
		if (rc) {
			CAM_WARN(CAM_SENSOR,
			"BoB PWM setup failed rc: %d", rc);
			rc = 0;
		}
	}

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "power up the core is failed:%d", rc);
		return rc;
	}

	rc = camera_io_init(&(s_ctrl->io_master_info));
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "cci_init failed: rc: %d", rc);

    /*added by webber.wang for Bug 55091 - [neo2.5]four 6dof camera sensors debug--start*/   
    #ifdef NEO25_FOUR_6DOF_CAMERAS_SUPPORT
	if(s_ctrl->sensordata->slave_info.sensor_id == 0x6710)/*Modify by PICO_Driver sophia.wang for bug1178: bringup eyetracking camera in xr2 cv3*/
	{
       cam_change_sensor_private_slave_address(s_ctrl);
	}
    #endif
    /*added by webber.wang for Bug 55091 - [neo2.5]four 6dof camera sensors debug--end*/
	return rc;
}

int cam_sensor_power_down(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info *soc_info;
	int rc = 0;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: s_ctrl %pK", s_ctrl);
		return -EINVAL;
	}
	CAM_INFO(CAM_SENSOR, "enter");

	power_info = &s_ctrl->sensordata->power_info;
	soc_info = &s_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_SENSOR, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "power down the core is failed:%d", rc);
		return rc;
	}

	if (s_ctrl->bob_pwm_switch) {
		rc = cam_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, false);
		if (rc) {
			CAM_WARN(CAM_SENSOR,
				"BoB PWM setup failed rc: %d", rc);
			rc = 0;
		}
	}

	camera_io_release(&(s_ctrl->io_master_info));
	return rc;
}

int cam_sensor_apply_settings(struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id, enum cam_sensor_packet_opcodes opcode)
{
	int rc = 0, offset, i;
	uint64_t top = 0, del_req_id = 0;
	struct i2c_settings_array *i2c_set = NULL;
	struct i2c_settings_list *i2c_list;

	if (req_id == 0) {
		switch (opcode) {
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
			i2c_set = &s_ctrl->i2c_data.streamon_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
			i2c_set = &s_ctrl->i2c_data.init_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
			i2c_set = &s_ctrl->i2c_data.config_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
			i2c_set = &s_ctrl->i2c_data.streamoff_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_READ: {
			i2c_set = &s_ctrl->i2c_data.read_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_POWERON_REG: {
			i2c_set = &s_ctrl->i2c_data.poweron_reg_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_POWEROFF_REG: {
			i2c_set = &s_ctrl->i2c_data.poweroff_reg_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE:
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE:
		default:
			return 0;
		}
		if (i2c_set->is_settings_valid == 1) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head), list) {
				rc = cam_sensor_i2c_modes_util(s_ctrl,
					i2c_list);
/*Add by PICO_Driver zhaohaiyun for bug5084669: OV680 I2C write fail start*/
#ifdef __CAMERA_EDITOR__
				if(rc < 0 && (s_ctrl->io_master_info.cci_client->sid == 0x35)){
					for(i = 0; i < 5; i++){
						CAM_ERR(CAM_SENSOR,
							"debug_OV680 Failed to apply reg address: 0x%x,index : %d",
							i2c_list->i2c_settings.reg_setting[0],i);
						rc = cam_sensor_i2c_modes_util(s_ctrl,
							i2c_list);
						if(rc == 0){
							break;
						}
					}
				}
#endif
/*Add by PICO_Driver zhaohaiyun for bug5084669: OV680 I2C write fail end*/
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"Failed to apply settings: %d",
						rc);
					goto EXIT_RESTORE;
				}
			}
		}
	} else {
		offset = req_id % MAX_PER_FRAME_ARRAY;
		i2c_set = &(s_ctrl->i2c_data.per_frame[offset]);
		if (i2c_set->is_settings_valid == 1 &&
			i2c_set->request_id == req_id) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head), list) {
				rc = cam_sensor_i2c_modes_util(s_ctrl,
					i2c_list);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"Failed to apply settings: %d",
						rc);
					goto EXIT_RESTORE;
				}
			}
		} else {
			CAM_DBG(CAM_SENSOR,
				"Invalid/NOP request to apply: %lld", req_id);
		}

		/* Change the logic dynamically */
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((req_id >=
				s_ctrl->i2c_data.per_frame[i].request_id) &&
				(top <
				s_ctrl->i2c_data.per_frame[i].request_id) &&
				(s_ctrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				del_req_id = top;
				top = s_ctrl->i2c_data.per_frame[i].request_id;
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			goto EXIT_RESTORE;

		CAM_DBG(CAM_SENSOR, "top: %llu, del_req_id:%llu",
			top, del_req_id);

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((del_req_id >
				 s_ctrl->i2c_data.per_frame[i].request_id) && (
				 s_ctrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				s_ctrl->i2c_data.per_frame[i].request_id = 0;
				rc = delete_request(
					&(s_ctrl->i2c_data.per_frame[i]));
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"Delete request Fail:%lld rc:%d",
						del_req_id, rc);
			}
		}
	}

EXIT_RESTORE:
	(void)cam_sensor_restore_slave_info(s_ctrl);

	return rc;
}

int32_t cam_sensor_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!apply)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}
	CAM_DBG(CAM_REQ, " Sensor update req id: %lld", apply->request_id);
	trace_cam_apply_req("Sensor", apply->request_id);
	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	rc = cam_sensor_apply_settings(s_ctrl, apply->request_id,
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int32_t cam_sensor_flush_request(struct cam_req_mgr_flush_request *flush_req)
{
	int32_t rc = 0, i;
	uint32_t cancel_req_id_found = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct i2c_settings_array *i2c_set = NULL;

	if (!flush_req)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(flush_req->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if ((s_ctrl->sensor_state != CAM_SENSOR_START) &&
		(s_ctrl->sensor_state != CAM_SENSOR_CONFIG)) {
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return rc;
	}

	if (s_ctrl->i2c_data.per_frame == NULL) {
		CAM_ERR(CAM_SENSOR, "i2c frame data is NULL");
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return -EINVAL;
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		s_ctrl->last_flush_req = flush_req->req_id;
		CAM_DBG(CAM_SENSOR, "last reqest to flush is %lld",
			flush_req->req_id);
	}

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
		i2c_set = &(s_ctrl->i2c_data.per_frame[i]);

		if ((flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ)
				&& (i2c_set->request_id != flush_req->req_id))
			continue;

		if (i2c_set->is_settings_valid == 1) {
			rc = delete_request(i2c_set);
			if (rc < 0)
				CAM_ERR(CAM_SENSOR,
					"delete request: %lld rc: %d",
					i2c_set->request_id, rc);

			if (flush_req->type ==
				CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
				cancel_req_id_found = 1;
				break;
			}
		}
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ &&
		!cancel_req_id_found)
		CAM_DBG(CAM_SENSOR,
			"Flush request id:%lld not found in the pending list",
			flush_req->req_id);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}
