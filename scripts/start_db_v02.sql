-- MySQL dump for office_robot_db v02
-- Schema based on start_db.sql (updated for AI Scenario & User Location Mapping)
-- Data populated with examples

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `Locations`
-- (Users 테이블에서 참조하기 위해 먼저 정의)

DROP TABLE IF EXISTS `Locations`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Locations` (
  `location_id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) DEFAULT NULL,
  `type` enum('OFFICE','MEETING_ROOM','WAREHOUSE','CHARGER','WAITING_AREA') DEFAULT NULL COMMENT '시설물 종류 (SR-015)',
  `coordinate_x` float DEFAULT NULL,
  `coordinate_y` float DEFAULT NULL,
  `theta` float DEFAULT NULL,
  `is_restricted` tinyint(1) DEFAULT '0' COMMENT '금지 구역 여부 (SR-014)',
  PRIMARY KEY (`location_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='주요 목적지 및 POI 데이터 (SR-015)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Locations`
--

LOCK TABLES `Locations` WRITE;
/*!40000 ALTER TABLE `Locations` DISABLE KEYS */;
INSERT INTO `Locations` (`name`, `type`, `coordinate_x`, `coordinate_y`, `theta`, `is_restricted`) VALUES
('office_1', 'OFFICE', 0.5, -0.3, 0.0, 0),
('office_2', 'OFFICE', 1.0, -0.3, 0.0, 0),
('office_3', 'OFFICE', 1.5, -0.3, 0.0, 0),
('snack_entrance', 'WAREHOUSE', 1.8, -1.0, 0.0, 0),
('small_meeting_room', 'MEETING_ROOM', 0.5, -1.5, 0.0, 0),
('large_meeting_room', 'MEETING_ROOM', 1.5, -1.5, 0.0, 0),
('charger_1', 'CHARGER', 0.1, -2.2, 0.0, 0),
('charger_2', 'CHARGER', 1.8, -2.2, 0.0, 0),
('waiting_area', 'WAITING_AREA', 0.0, 0.0, 0.0, 0);
/*!40000 ALTER TABLE `Locations` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Users`
--

DROP TABLE IF EXISTS `Users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Users` (
  `user_id` int NOT NULL AUTO_INCREMENT,
  `account` varchar(255) NOT NULL,
  `password_hash` varchar(255) NOT NULL,
  `name` varchar(255) DEFAULT NULL,
  `department` varchar(255) DEFAULT NULL,
  `role` enum('STAFF','ADMIN') DEFAULT 'STAFF',
  `location_id` int DEFAULT NULL COMMENT '주 근무 위치 (오피스 등)',
  `app_token` varchar(255) DEFAULT NULL COMMENT '앱 푸시 알림용 토큰 (SR-007)',
  `created_at` timestamp NULL DEFAULT (now()),
  PRIMARY KEY (`user_id`),
  UNIQUE KEY `account` (`account`),
  KEY `location_id` (`location_id`),
  CONSTRAINT `Users_ibfk_1` FOREIGN KEY (`location_id`) REFERENCES `Locations` (`location_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='내부 직원 정보 관리 (SR-009)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Users`
--

LOCK TABLES `Users` WRITE;
/*!40000 ALTER TABLE `Users` DISABLE KEYS */;
-- location_id 매핑: admin(1) -> office_1(1), alice(3) -> office_2(2), bob(4) -> office_3(3)
INSERT INTO `Users` (`user_id`, `account`, `password_hash`, `name`, `department`, `role`, `location_id`, `app_token`, `created_at`) VALUES 
(1, 'admin', '1234', '관리자', '운영팀', 'ADMIN', 1, NULL, NOW()),
(2, 'staff', '1234', '스태프1', '기술팀', 'STAFF', 2, NULL, NOW()),
(3, 'alice', '1234', '앨리스', '인사팀', 'STAFF', 2, NULL, NOW()),
(4, 'bob', '1234', '밥', '개발팀', 'STAFF', 3, NULL, NOW());
/*!40000 ALTER TABLE `Users` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Visitors`
--

DROP TABLE IF EXISTS `Visitors`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Visitors` (
  `visitor_id` int NOT NULL AUTO_INCREMENT,
  `host_user_id` int DEFAULT NULL COMMENT '담당 내부 직원 (SR-005)',
  `name` varchar(255) DEFAULT NULL,
  `qr_code` varchar(255) DEFAULT NULL COMMENT '인증용 QR 데이터',
  `status` enum('PENDING','VERIFIED','GUIDED','REJECTED') DEFAULT 'PENDING',
  `visit_date` datetime DEFAULT NULL,
  PRIMARY KEY (`visitor_id`),
  UNIQUE KEY `qr_code` (`qr_code`),
  KEY `host_user_id` (`host_user_id`),
  CONSTRAINT `Visitors_ibfk_1` FOREIGN KEY (`host_user_id`) REFERENCES `Users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='방문객 및 인증 정보 (SR-005)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Visitors`
--

LOCK TABLES `Visitors` WRITE;
/*!40000 ALTER TABLE `Visitors` DISABLE KEYS */;
INSERT INTO `Visitors` (`host_user_id`, `name`, `qr_code`, `status`, `visit_date`) VALUES
(2, '방문객1', 'QR_VISIT_001', 'PENDING', NOW()),
(3, '방문객2', 'QR_VISIT_002', 'VERIFIED', NOW());
/*!40000 ALTER TABLE `Visitors` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Map_Zones`
--

DROP TABLE IF EXISTS `Map_Zones`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Map_Zones` (
  `zone_id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) DEFAULT NULL,
  `polygon_data` text COMMENT '구역 좌표 데이터 (JSON/WKT)',
  `type` varchar(255) DEFAULT NULL COMMENT 'RESTRICTED, SLOW_ZONE etc.',
  `active` tinyint(1) DEFAULT '1',
  PRIMARY KEY (`zone_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='금지 구역 및 특수 구역 정의 (SR-014)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Map_Zones`
--

LOCK TABLES `Map_Zones` WRITE;
/*!40000 ALTER TABLE `Map_Zones` DISABLE KEYS */;
INSERT INTO `Map_Zones` (`name`, `polygon_data`, `type`, `active`) VALUES
('Restricted Area A', '[[0,0], [1,0], [1,1], [0,1]]', 'RESTRICTED', 1),
('Slow Zone B', '[[3,3], [4,3], [4,4], [3,4]]', 'SLOW_ZONE', 1);
/*!40000 ALTER TABLE `Map_Zones` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `IoT_Devices`
--

DROP TABLE IF EXISTS `IoT_Devices`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `IoT_Devices` (
  `device_id` int NOT NULL AUTO_INCREMENT,
  `location_id` int DEFAULT NULL,
  `type` varchar(255) DEFAULT NULL COMMENT 'LIGHT, HVAC',
  `status` varchar(255) DEFAULT NULL COMMENT 'ON/OFF, Temperature',
  PRIMARY KEY (`device_id`),
  KEY `location_id` (`location_id`),
  CONSTRAINT `IoT_Devices_ibfk_1` FOREIGN KEY (`location_id`) REFERENCES `Locations` (`location_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='시설물 환경 제어 장치 (SR-016)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `IoT_Devices`
--

LOCK TABLES `IoT_Devices` WRITE;
/*!40000 ALTER TABLE `IoT_Devices` DISABLE KEYS */;
INSERT INTO `IoT_Devices` (`location_id`, `type`, `status`) VALUES
(1, 'LIGHT', 'ON'),
(2, 'HVAC', '24C');
/*!40000 ALTER TABLE `IoT_Devices` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Robots`
--

DROP TABLE IF EXISTS `Robots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Robots` (
  `robot_id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) DEFAULT NULL,
  `status` enum('IDLE','WAITING','ASSIGNED','MOVING','GUIDING','CHARGING','ERROR','OFFLINE') DEFAULT 'IDLE' COMMENT '로봇 상태 (SR-008, SR-010)',
  `battery_level` float DEFAULT NULL COMMENT '배터리 잔량 % (SR-010, SR-017)',
  `current_x` float DEFAULT NULL,
  `current_y` float DEFAULT NULL,
  `current_task_id` int DEFAULT NULL,
  `last_heartbeat` timestamp NULL DEFAULT NULL,
  PRIMARY KEY (`robot_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='로봇 상태 및 실시간 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Robots`
--

LOCK TABLES `Robots` WRITE;
/*!40000 ALTER TABLE `Robots` DISABLE KEYS */;
INSERT INTO `Robots` (`name`, `status`, `battery_level`, `current_x`, `current_y`, `current_task_id`, `last_heartbeat`) VALUES
('robot_01', 'IDLE', 100.0, 0.1, 0.1, NULL, NOW());
/*!40000 ALTER TABLE `Robots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Products`
--

DROP TABLE IF EXISTS `Products`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Products` (
  `product_id` int NOT NULL AUTO_INCREMENT,
  `name` varchar(255) DEFAULT NULL,
  `type` varchar(255) DEFAULT NULL COMMENT 'SNACK, ITEM',
  `image_url` varchar(255) DEFAULT NULL COMMENT '객체 인식용 참조 이미지 (SR-006)',
  `stock_quantity` int DEFAULT NULL COMMENT '재고 관리 (SR-007)',
  PRIMARY KEY (`product_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='간식 및 배송 물품 정보';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Products`
--

LOCK TABLES `Products` WRITE;
/*!40000 ALTER TABLE `Products` DISABLE KEYS */;
INSERT INTO `Products` (`name`, `type`, `stock_quantity`, `image_url`) VALUES
('Choco Pie (초코파이)', 'SNACK', 50, 'img/chocopie.png'),
('Coffee (커피)', 'SNACK', 100, 'img/coffee.png'),
('Orange Juice (오렌지 주스)', 'SNACK', 20, 'img/juice.png'),
('A4 Paper', 'ITEM', 500, NULL),
('Stapler', 'ITEM', 5, NULL);
/*!40000 ALTER TABLE `Products` ENABLE KEYS */;
UNLOCK TABLES;


DROP TABLE IF EXISTS 'reservations';
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE IF 'reservations' (
  'id' INT AUTO_INCREMENT PRIMARY KEY,

  -- 담당자 정보 (쿠키의 user_account와 user_name에 대응)
  'manager_account' VARCHAR(50) NOT NULL COMMENT '담당자 아이디 (user.account)',
  'manager_name' VARCHAR(30) NOT NULL COMMENT '담당자 이름 (user.name)',

  -- 방문객 정보
  'visitor_name' VARCHAR(30) NOT NULL,
  'visitor_phone' VARCHAR(15) NOT NULL,
  'purpose' VARCHAR(100),

  -- 예약 일시
  'visit_date' DATE NOT NULL,
  'visit_time' TIME,

  -- 상태 및 생성일
  'status' ENUM('PENDING', 'APPROVED', 'REJECTED') DEFAULT 'PENDING',
  'created_at' TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Table structure for table `Tasks`
--

DROP TABLE IF EXISTS `Tasks`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Tasks` (
  `task_id` int NOT NULL AUTO_INCREMENT,
  `requester_id` int DEFAULT NULL COMMENT '요청자',
  `receiver_id` int DEFAULT NULL COMMENT '수신자 (물품 배송 시)',
  `assigned_robot_id` int DEFAULT NULL COMMENT '스마트 배차 (SR-011)',
  `task_type` enum('GUIDE_GUEST','SNACK_DELIVERY','ITEM_DELIVERY','RETURN','PATROL','MANUAL_MOVE') DEFAULT NULL,
  `priority` int DEFAULT '3' COMMENT '1:가이드, 2:물품, 3:간식 (SR-012)',
  `status` enum('PENDING','ASSIGNED','IN_PROGRESS','ARRIVED','COMPLETED','FAILED','CANCELLED') DEFAULT 'PENDING',
  `destination_id` int DEFAULT NULL,
  `target_location_name` varchar(255) DEFAULT NULL,
  `visitor_id` int DEFAULT NULL COMMENT '가이드 대상일 경우',
  `details` json DEFAULT NULL COMMENT '세부 정보 (JSON)',
  `created_at` timestamp NULL DEFAULT (now()),
  `completed_at` timestamp NULL DEFAULT NULL,
  PRIMARY KEY (`task_id`),
  KEY `requester_id` (`requester_id`),
  KEY `receiver_id` (`receiver_id`),
  KEY `assigned_robot_id` (`assigned_robot_id`),
  KEY `destination_id` (`destination_id`),
  KEY `visitor_id` (`visitor_id`),
  CONSTRAINT `Tasks_ibfk_1` FOREIGN KEY (`requester_id`) REFERENCES `Users` (`user_id`),
  CONSTRAINT `Tasks_ibfk_2` FOREIGN KEY (`receiver_id`) REFERENCES `Users` (`user_id`),
  CONSTRAINT `Tasks_ibfk_3` FOREIGN KEY (`assigned_robot_id`) REFERENCES `Robots` (`robot_id`),
  CONSTRAINT `Tasks_ibfk_4` FOREIGN KEY (`destination_id`) REFERENCES `Locations` (`location_id`),
  CONSTRAINT `Tasks_ibfk_5` FOREIGN KEY (`visitor_id`) REFERENCES `Visitors` (`visitor_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='로봇이 수행해야 할 임무 (SR-011, SR-012)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Tasks`
--

LOCK TABLES `Tasks` WRITE;
/*!40000 ALTER TABLE `Tasks` DISABLE KEYS */;
-- No initial tasks
/*!40000 ALTER TABLE `Tasks` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Task_Items`
--

DROP TABLE IF EXISTS `Task_Items`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Task_Items` (
  `task_item_id` int NOT NULL AUTO_INCREMENT,
  `task_id` int DEFAULT NULL,
  `product_id` int DEFAULT NULL,
  `quantity` int DEFAULT NULL,
  PRIMARY KEY (`task_item_id`),
  KEY `task_id` (`task_id`),
  KEY `product_id` (`product_id`),
  CONSTRAINT `Task_Items_ibfk_1` FOREIGN KEY (`task_id`) REFERENCES `Tasks` (`task_id`),
  CONSTRAINT `Task_Items_ibfk_2` FOREIGN KEY (`product_id`) REFERENCES `Products` (`product_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='배달 임무 상세 물품';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Task_Items`
--

LOCK TABLES `Task_Items` WRITE;
/*!40000 ALTER TABLE `Task_Items` DISABLE KEYS */;
/*!40000 ALTER TABLE `Task_Items` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `System_Logs`
--

DROP TABLE IF EXISTS `System_Logs`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `System_Logs` (
  `log_id` int NOT NULL AUTO_INCREMENT,
  `timestamp` timestamp NULL DEFAULT (now()),
  `log_level` varchar(255) DEFAULT NULL COMMENT 'INFO, WARN, ERROR',
  `event_type` varchar(255) DEFAULT NULL COMMENT 'OBSTACLE, TRAFFIC, AUTH_FAIL (SR-007)',
  `robot_id` int DEFAULT NULL,
  `message` text,
  PRIMARY KEY (`log_id`),
  KEY `robot_id` (`robot_id`),
  CONSTRAINT `System_Logs_ibfk_1` FOREIGN KEY (`robot_id`) REFERENCES `Robots` (`robot_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='시스템 에러 및 이벤트 로그 (SR-018)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `System_Logs`
--

LOCK TABLES `System_Logs` WRITE;
/*!40000 ALTER TABLE `System_Logs` DISABLE KEYS */;
/*!40000 ALTER TABLE `System_Logs` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Robot_Telemetry_Logs`
--

DROP TABLE IF EXISTS `Robot_Telemetry_Logs`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Robot_Telemetry_Logs` (
  `telemetry_id` int NOT NULL AUTO_INCREMENT,
  `robot_id` int DEFAULT NULL,
  `timestamp` timestamp NULL DEFAULT NULL,
  `battery_level` float DEFAULT NULL,
  `location_x` float DEFAULT NULL,
  `location_y` float DEFAULT NULL,
  `status` enum('IDLE','WAITING','ASSIGNED','MOVING','GUIDING','CHARGING','ERROR','OFFLINE') DEFAULT NULL,
  PRIMARY KEY (`telemetry_id`),
  KEY `robot_id` (`robot_id`),
  CONSTRAINT `Robot_Telemetry_Logs_ibfk_1` FOREIGN KEY (`robot_id`) REFERENCES `Robots` (`robot_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='로봇 주행 및 상태 이력 (SR-018)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Robot_Telemetry_Logs`
--

LOCK TABLES `Robot_Telemetry_Logs` WRITE;
/*!40000 ALTER TABLE `Robot_Telemetry_Logs` DISABLE KEYS */;
/*!40000 ALTER TABLE `Robot_Telemetry_Logs` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Notification_Logs`
--

DROP TABLE IF EXISTS `Notification_Logs`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Notification_Logs` (
  `notification_id` int NOT NULL AUTO_INCREMENT,
  `user_id` int DEFAULT NULL,
  `event_type` varchar(255) DEFAULT NULL COMMENT 'ARRIVED, NO_STOCK etc.',
  `message` text,
  `sent_at` timestamp NULL DEFAULT (now()),
  `is_read` tinyint(1) DEFAULT '0',
  PRIMARY KEY (`notification_id`),
  KEY `user_id` (`user_id`),
  CONSTRAINT `Notification_Logs_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `Users` (`user_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='앱 알림 발송 이력 (SR-007)';
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Notification_Logs`
--

LOCK TABLES `Notification_Logs` WRITE;
/*!40000 ALTER TABLE `Notification_Logs` DISABLE KEYS */;
/*!40000 ALTER TABLE `Notification_Logs` ENABLE KEYS */;
UNLOCK TABLES;

/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed
