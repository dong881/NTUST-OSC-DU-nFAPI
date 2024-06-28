/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SemiStaticChannelAccessConfigUE_r17_H_
#define	_SemiStaticChannelAccessConfigUE_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SemiStaticChannelAccessConfigUE_r17__periodUE_r17 {
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_ms1	= 0,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_ms2	= 1,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_ms2dot5	= 2,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_ms4	= 3,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_ms5	= 4,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_ms10	= 5,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_spare2	= 6,
	SemiStaticChannelAccessConfigUE_r17__periodUE_r17_spare1	= 7
} e_SemiStaticChannelAccessConfigUE_r17__periodUE_r17;

/* SemiStaticChannelAccessConfigUE-r17 */
typedef struct SemiStaticChannelAccessConfigUE_r17 {
	long	 periodUE_r17;
	long	 offsetUE_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SemiStaticChannelAccessConfigUE_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_periodUE_r17_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SemiStaticChannelAccessConfigUE_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SemiStaticChannelAccessConfigUE_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SemiStaticChannelAccessConfigUE_r17_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SemiStaticChannelAccessConfigUE_r17_H_ */
#include <asn_internal.h>
