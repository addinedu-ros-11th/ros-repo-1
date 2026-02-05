CREATE TABLE `Users` (
  `user_id` int PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `department` varchar(255),
  `role` ENUM ('STAFF', 'ADMIN') DEFAULT 'STAFF',
  `app_token` varchar(255) COMMENT '앱 푸시 알림용 토큰 (SR-007)',
  `created_at` timestamp DEFAULT (now())
);

CREATE TABLE `Visitors` (
  `visitor_id` int PRIMARY KEY AUTO_INCREMENT,
  `host_user_id` int COMMENT '담당 내부 직원 (SR-005)',
  `name` varchar(255),
  `qr_code` varchar(255) UNIQUE COMMENT '인증용 QR 데이터',
  `status` ENUM ('PENDING', 'VERIFIED', 'GUIDED', 'REJECTED') DEFAULT 'PENDING',
  `visit_date` datetime
);

CREATE TABLE `Locations` (
  `location_id` int PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `type` ENUM ('OFFICE', 'MEETING_ROOM', 'WAREHOUSE', 'CHARGER', 'WAITING_AREA') COMMENT '시설물 종류 (SR-015)',
  `coordinate_x` float,
  `coordinate_y` float,
  `is_restricted` boolean DEFAULT false COMMENT '금지 구역 여부 (SR-014)'
);

CREATE TABLE `Map_Zones` (
  `zone_id` int PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `polygon_data` text COMMENT '구역 좌표 데이터 (JSON/WKT)',
  `type` varchar(255) COMMENT 'RESTRICTED, SLOW_ZONE etc.',
  `active` boolean DEFAULT true
);

CREATE TABLE `IoT_Devices` (
  `device_id` int PRIMARY KEY AUTO_INCREMENT,
  `location_id` int,
  `type` varchar(255) COMMENT 'LIGHT, HVAC',
  `status` varchar(255) COMMENT 'ON/OFF, Temperature'
);

CREATE TABLE `Robots` (
  `robot_id` int PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `status` ENUM ('IDLE', 'ASSIGNED', 'MOVING', 'GUIDING', 'CHARGING', 'ERROR') DEFAULT 'IDLE' COMMENT '로봇 상태 (SR-008, SR-010)',
  `battery_level` int COMMENT '배터리 잔량 % (SR-010, SR-017)',
  `current_x` float,
  `current_y` float,
  `last_heartbeat` timestamp
);

CREATE TABLE `Products` (
  `product_id` int PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `type` varchar(255) COMMENT 'SNACK, ITEM',
  `image_url` varchar(255) COMMENT '객체 인식용 참조 이미지 (SR-006)',
  `stock_quantity` int COMMENT '재고 관리 (SR-007)'
);

CREATE TABLE `Tasks` (
  `task_id` int PRIMARY KEY AUTO_INCREMENT,
  `requester_id` int COMMENT '요청자',
  `receiver_id` int COMMENT '수신자 (물품 배송 시)',
  `assigned_robot_id` int COMMENT '스마트 배차 (SR-011)',
  `task_type` ENUM ('GUIDE', 'SNACK_DELIVERY', 'ITEM_DELIVERY', 'RETURN'),
  `priority` int DEFAULT 3 COMMENT '1:가이드, 2:물품, 3:간식 (SR-012)',
  `status` ENUM ('WAITING', 'ASSIGNED', 'IN_PROGRESS', 'ARRIVED', 'COMPLETED', 'FAILED', 'CANCELLED') DEFAULT 'WAITING',
  `destination_id` int,
  `visitor_id` int COMMENT '가이드 대상일 경우',
  `created_at` timestamp DEFAULT (now()),
  `completed_at` timestamp
);

CREATE TABLE `Task_Items` (
  `task_item_id` int PRIMARY KEY AUTO_INCREMENT,
  `task_id` int,
  `product_id` int,
  `quantity` int
);

CREATE TABLE `System_Logs` (
  `log_id` int PRIMARY KEY AUTO_INCREMENT,
  `timestamp` timestamp DEFAULT (now()),
  `log_level` varchar(255) COMMENT 'INFO, WARN, ERROR',
  `event_type` varchar(255) COMMENT 'OBSTACLE, TRAFFIC, AUTH_FAIL (SR-007)',
  `robot_id` int,
  `message` text
);

CREATE TABLE `Robot_Telemetry_Logs` (
  `telemetry_id` int PRIMARY KEY AUTO_INCREMENT,
  `robot_id` int,
  `timestamp` timestamp,
  `battery_level` int,
  `location_x` float,
  `location_y` float,
  `status` ENUM ('IDLE', 'ASSIGNED', 'MOVING', 'GUIDING', 'CHARGING', 'ERROR')
);

CREATE TABLE `Notification_Logs` (
  `notification_id` int PRIMARY KEY AUTO_INCREMENT,
  `user_id` int,
  `event_type` varchar(255) COMMENT 'ARRIVED, NO_STOCK etc.',
  `message` text,
  `sent_at` timestamp DEFAULT (now()),
  `is_read` boolean DEFAULT false
);

ALTER TABLE `Users` COMMENT = '내부 직원 정보 관리 (SR-009)';

ALTER TABLE `Visitors` COMMENT = '방문객 및 인증 정보 (SR-005)';

ALTER TABLE `Locations` COMMENT = '주요 목적지 및 POI 데이터 (SR-015)';

ALTER TABLE `Map_Zones` COMMENT = '금지 구역 및 특수 구역 정의 (SR-014)';

ALTER TABLE `IoT_Devices` COMMENT = '시설물 환경 제어 장치 (SR-016)';

ALTER TABLE `Robots` COMMENT = '로봇 상태 및 실시간 정보';

ALTER TABLE `Products` COMMENT = '간식 및 배송 물품 정보';

ALTER TABLE `Tasks` COMMENT = '로봇이 수행해야 할 임무 (SR-011, SR-012)';

ALTER TABLE `Task_Items` COMMENT = '배달 임무 상세 물품';

ALTER TABLE `System_Logs` COMMENT = '시스템 에러 및 이벤트 로그 (SR-018)';

ALTER TABLE `Robot_Telemetry_Logs` COMMENT = '로봇 주행 및 상태 이력 (SR-018)';

ALTER TABLE `Notification_Logs` COMMENT = '앱 알림 발송 이력 (SR-007)';

ALTER TABLE `Visitors` ADD FOREIGN KEY (`host_user_id`) REFERENCES `Users` (`user_id`);

ALTER TABLE `IoT_Devices` ADD FOREIGN KEY (`location_id`) REFERENCES `Locations` (`location_id`);

ALTER TABLE `Tasks` ADD FOREIGN KEY (`requester_id`) REFERENCES `Users` (`user_id`);

ALTER TABLE `Tasks` ADD FOREIGN KEY (`receiver_id`) REFERENCES `Users` (`user_id`);

ALTER TABLE `Tasks` ADD FOREIGN KEY (`assigned_robot_id`) REFERENCES `Robots` (`robot_id`);

ALTER TABLE `Tasks` ADD FOREIGN KEY (`destination_id`) REFERENCES `Locations` (`location_id`);

ALTER TABLE `Tasks` ADD FOREIGN KEY (`visitor_id`) REFERENCES `Visitors` (`visitor_id`);

ALTER TABLE `Task_Items` ADD FOREIGN KEY (`task_id`) REFERENCES `Tasks` (`task_id`);

ALTER TABLE `Task_Items` ADD FOREIGN KEY (`product_id`) REFERENCES `Products` (`product_id`);

ALTER TABLE `System_Logs` ADD FOREIGN KEY (`robot_id`) REFERENCES `Robots` (`robot_id`);

ALTER TABLE `Robot_Telemetry_Logs` ADD FOREIGN KEY (`robot_id`) REFERENCES `Robots` (`robot_id`);

ALTER TABLE `Notification_Logs` ADD FOREIGN KEY (`user_id`) REFERENCES `Users` (`user_id`);
