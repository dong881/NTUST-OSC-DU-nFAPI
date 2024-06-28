/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PC5-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasurementReportSidelink_H_
#define	_MeasurementReportSidelink_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasurementReportSidelink__criticalExtensions_PR {
	MeasurementReportSidelink__criticalExtensions_PR_NOTHING,	/* No components present */
	MeasurementReportSidelink__criticalExtensions_PR_measurementReportSidelink_r16,
	MeasurementReportSidelink__criticalExtensions_PR_criticalExtensionsFuture
} MeasurementReportSidelink__criticalExtensions_PR;

/* Forward declarations */
struct MeasurementReportSidelink_r16_IEs;

/* MeasurementReportSidelink */
typedef struct MeasurementReportSidelink {
	struct MeasurementReportSidelink__criticalExtensions {
		MeasurementReportSidelink__criticalExtensions_PR present;
		union MeasurementReportSidelink__criticalExtensions_u {
			struct MeasurementReportSidelink_r16_IEs	*measurementReportSidelink_r16;
			struct MeasurementReportSidelink__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasurementReportSidelink_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasurementReportSidelink;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasurementReportSidelink_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasurementReportSidelink_1[1];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasurementReportSidelink-r16-IEs.h"

#endif	/* _MeasurementReportSidelink_H_ */
#include <asn_internal.h>
